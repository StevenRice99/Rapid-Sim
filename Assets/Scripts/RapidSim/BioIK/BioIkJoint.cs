using System;
using Unity.Mathematics;
using UnityEngine;

namespace RapidSim.BioIK
{
	[DisallowMultipleComponent]
	public class BioIkJoint : MonoBehaviour
	{
		[Header("Chain Properties")]
		[Tooltip("The parent joint.")]
		public BioIkJoint parent;
		
		[Tooltip("The child joint.")]
		public BioIkJoint child;

		[Header("Movement Properties")]
		[Tooltip("The motion along the X axis.")]
		public Motion x;
		
		[Tooltip("The motion along the Y axis.")]
		public Motion y;
		
		[Tooltip("The motion along the Z axis.")]
		public Motion z;
		
		[Tooltip("True if this joint is rotational, false if it is Prismatic.")]
		public bool rotational = true;
		
		private Vector3 _orientation = Vector3.zero;
		
		private float _dpx, _dpy, _dpz, _drx, _dry, _drz, _drw;

		private double _r1, _r2, _r3, _r4, _r5, _r6, _r7, _r8, _r9;

		public void Setup()
		{
			x = new(this, Vector3.right);
			y = new(this, Vector3.up);
			z = new(this, Vector3.forward);

			Transform t = transform;
			SetDefaultFrame(t.localPosition, t.localRotation);

			Vector3 forward = parent == null
				? Vector3.zero
				: Quaternion.Inverse(transform.localRotation) * transform.localPosition;

			SetOrientation(forward.magnitude != 0f ? Quaternion.LookRotation(forward, Vector3.up).eulerAngles : _orientation);
		}

		public void UpdateData()
		{
			_r1 = 1.0 - 2.0 * (_dry * _dry + _drz * _drz);
			_r2 = 2.0 * (_drx * _dry + _drw * _drz);
			_r3 = 2.0 * (_drx * _drz - _drw * _dry);
			_r4 = 2.0 * (_drx * _dry - _drw * _drz);
			_r5 = 1.0 - 2.0 * (_drx * _drx + _drz * _drz);
			_r6 = 2.0 * (_dry * _drz + _drw * _drx);
			_r7 = 2.0 * (_drx * _drz + _drw * _dry);
			_r8 = 2.0 * (_dry * _drz - _drw * _drx);
			_r9 = 1.0 - 2.0 * (_drx * _drx + _dry * _dry);
		}

		//Fast implementation to compute the local transform given the joint values (in radians / metres)
		public void ComputeLocalTransformation(double valueX, double valueY, double valueZ, out double lpX, out double lpY, out double lpZ, out double lrX, out double lrY, out double lrZ, out double lrW)
		{
			if (!rotational)
			{
				double axisX = valueX * x.axis.x + valueY * y.axis.x + valueZ * z.axis.x;
				double axisY = valueX * x.axis.y + valueY * y.axis.y + valueZ * z.axis.y;
				double axisZ = valueX * x.axis.z + valueY * y.axis.z + valueZ * z.axis.z;
				//Local position for translational motion
				lpX = _dpx + _r1 * axisX + _r4 * axisY + _r7 * axisZ;
				lpY = _dpy + _r2 * axisX + _r5 * axisY + _r8 * axisZ;
				lpZ = _dpz + _r3 * axisX + _r6 * axisY + _r9 * axisZ;
				//Local rotation for translational motion
				lrX = _drx; lrY = _dry; lrZ = _drz; lrW = _drw;
				return;
			}

			double sin, x1, y1, z1, w1, x2, y2, z2, w2, qx, qy, qz, qw;
			if (valueZ != 0.0)
			{
				sin = math.sin(valueZ / 2.0);
				qx = z.axis.x * sin;
				qy = z.axis.y * sin;
				qz = z.axis.z * sin;
				qw = math.cos(valueZ / 2.0);
				if (valueX != 0.0)
				{
					sin = math.sin(valueX / 2.0);
					x1 = x.axis.x * sin;
					y1 = x.axis.y * sin;
					z1 = x.axis.z * sin;
					w1 = math.cos(valueX / 2.0);
					x2 = qx; y2 = qy; z2 = qz; w2 = qw;
					qx = x1 * w2 + y1 * z2 - z1 * y2 + w1 * x2;
					qy = -x1 * z2 + y1 * w2 + z1 * x2 + w1 * y2;
					qz = x1 * y2 - y1 * x2 + z1 * w2 + w1 * z2;
					qw = -x1 * x2 - y1 * y2 - z1 * z2 + w1 * w2;
					if (valueY != 0.0)
					{
						sin = math.sin(valueY / 2.0);
						x1 = y.axis.x * sin;
						y1 = y.axis.y * sin;
						z1 = y.axis.z * sin;
						w1 = math.cos(valueY / 2.0);
						x2 = qx; y2 = qy; z2 = qz; w2 = qw;
						qx = x1 * w2 + y1 * z2 - z1 * y2 + w1 * x2;
						qy = -x1 * z2 + y1 * w2 + z1 * x2 + w1 * y2;
						qz = x1 * y2 - y1 * x2 + z1 * w2 + w1 * z2;
						qw = -x1 * x2 - y1 * y2 - z1 * z2 + w1 * w2;
					}
				}
				else if (valueY != 0.0)
				{
					sin = math.sin(valueY / 2.0);
					x1 = y.axis.x * sin;
					y1 = y.axis.y * sin;
					z1 = y.axis.z * sin;
					w1 = math.cos(valueY / 2.0);
					x2 = qx; y2 = qy; z2 = qz; w2 = qw;
					qx = x1 * w2 + y1 * z2 - z1 * y2 + w1 * x2;
					qy = -x1 * z2 + y1 * w2 + z1 * x2 + w1 * y2;
					qz = x1 * y2 - y1 * x2 + z1 * w2 + w1 * z2;
					qw = -x1 * x2 - y1 * y2 - z1 * z2 + w1 * w2;
				}
			}
			else if (valueX != 0.0)
			{
				sin = math.sin(valueX / 2.0);
				qx = x.axis.x * sin;
				qy = x.axis.y * sin;
				qz = x.axis.z * sin;
				qw = math.cos(valueX / 2.0);
				if (valueY != 0.0)
				{
					sin = math.sin(valueY / 2.0);
					x1 = y.axis.x * sin;
					y1 = y.axis.y * sin;
					z1 = y.axis.z * sin;
					w1 = math.cos(valueY / 2.0);
					x2 = qx; y2 = qy; z2 = qz; w2 = qw;
					qx = x1 * w2 + y1 * z2 - z1 * y2 + w1 * x2;
					qy = -x1 * z2 + y1 * w2 + z1 * x2 + w1 * y2;
					qz = x1 * y2 - y1 * x2 + z1 * w2 + w1 * z2;
					qw = -x1 * x2 - y1 * y2 - z1 * z2 + w1 * w2;
				}
			}
			else if(valueY != 0.0)
			{
				sin = math.sin(valueY / 2.0);
				qx = y.axis.x * sin;
				qy = y.axis.y * sin;
				qz = y.axis.z * sin;
				qw = math.cos(valueY / 2.0);
			}
			else
			{
				lpX = _dpx;
				lpY = _dpy;
				lpZ = _dpz;
				lrX = _drx;
				lrY = _dry;
				lrZ = _drz;
				lrW = _drw;
				return;
			}

			//Local Rotation
			//R' = R*Q
			lrX = _drx * qw + _dry * qz - _drz * qy + _drw * qx;
			lrY = -_drx * qz + _dry * qw + _drz * qx + _drw * qy;
			lrZ = _drx * qy - _dry * qx + _drz * qw + _drw * qz;
			lrW = -_drx * qx - _dry * qy - _drz * qz + _drw * qw;

			//Local Position
			lpX = _dpx;
			lpY = _dpy;
			lpZ = _dpz;
		}

		public void SetOrientation(Vector3 value)
		{
			_orientation = value;
			Quaternion o = Quaternion.Euler(_orientation);
			x.axis = o * Vector3.right;
			y.axis = o * Vector3.up;
			z.axis = o * Vector3.forward;
		}

		public int GetDoF()
		{
			int dof = 0;
			if (x.enabled)
			{
				dof += 1;
			}
			if (y.enabled)
			{
				dof += 1;
			}
			if (z.enabled)
			{
				dof += 1;
			}
			return dof;
		}

		private void SetDefaultFrame(Vector3 localPosition, Quaternion localRotation)
		{
			_dpx = localPosition.x;
			_dpy = localPosition.y;
			_dpz = localPosition.z;
			_drx = localRotation.x;
			_dry = localRotation.y;
			_drz = localRotation.z;
			_drw = localRotation.w;
		}

		[Serializable]
		public class Motion
		{
			[Tooltip("The joint this motion is attached to.")]
			public BioIkJoint joint;
			
			[Tooltip("The axis this movement controls.")]
			public Vector3 axis;

			[Tooltip("True if this joint has movement along this axis, false otherwise.")]
			public bool enabled;
			
			[Tooltip("The lower limit of this joint in meters (for prismatic joints) or radians (for rotational joints).")]
			public double lowerLimit;
			
			[Tooltip("The upper limit of this joint in meters (for prismatic joints) or radians (for rotational joints).")]
			public double upperLimit;

			[Tooltip("The target value for this joint in meters (for prismatic joints) or radians (for rotational joints).")]
			public double targetValue;

			public Motion(BioIkJoint joint, Vector3 axis)
			{
				this.joint = joint;
				this.axis = axis;
			}

			public double GetLowerLimit() => joint.rotational ? math.radians(lowerLimit) : lowerLimit;

			public double GetUpperLimit() => joint.rotational ? math.radians(upperLimit) : upperLimit;

			public void SetLowerLimit(double value)
			{
				lowerLimit = math.min(0.0, value);
			}

			public void SetUpperLimit(double value)
			{
				upperLimit = math.max(0.0, value);
			}

			public void SetTargetValue(double value)
			{
				targetValue = math.clamp(value, lowerLimit, upperLimit);
			}

			public double GetTargetValue()
			{
				return joint.rotational ? math.radians(targetValue) : targetValue;
			}
		}
	}
}