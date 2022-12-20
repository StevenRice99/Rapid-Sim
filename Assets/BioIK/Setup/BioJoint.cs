using BioIK.Helpers;
using Unity.Mathematics;
using UnityEngine;

namespace BioIK.Setup
{
	[AddComponentMenu("")]
	public class BioJoint : MonoBehaviour
	{
		public BioSegment segment;
		public Motion x,y,z;
		public bool rotational = true;

		[SerializeField] private Vector3 orientation = Vector3.zero;
		[SerializeField] private float dpx, dpy, dpz, drx, dry, drz, drw;

		private double _r1, _r2, _r3, _r4, _r5, _r6, _r7, _r8, _r9;

		private void OnEnable()
		{
			if (segment != null)
			{
				segment.controller.Refresh();
			}
		}

		private void OnDisable()
		{
			segment.controller.Refresh();
		}

		public BioJoint Create(BioSegment value)
		{
			segment = value;

			x = new(this, Vector3.right);
			y = new(this, Vector3.up);
			z = new(this, Vector3.forward);

			Transform t = transform;
			SetDefaultFrame(t.localPosition, t.localRotation);

			Vector3 forward = Vector3.zero;
			if (segment.child != null)
			{
				forward = segment.child.transform.localPosition;
			}
			else if(segment.parent != null)
			{
				forward = Quaternion.Inverse(segment.transform.localRotation) * segment.transform.localPosition;
			}

			SetOrientation(forward.magnitude != 0f
				? Quaternion.LookRotation(forward, Vector3.up).eulerAngles
				: orientation);

			return this;
		}

		public void Erase()
		{
			RestoreDefaultFrame();
			segment.transform.hideFlags = HideFlags.None;
			Destroy(this);
		}

		public void UpdateData()
		{
			_r1 = 1.0 - 2.0 * (dry * dry + drz * drz);
			_r2 = 2.0 * (drx * dry + drw * drz);
			_r3 = 2.0 * (drx * drz - drw * dry);
			_r4 = 2.0 * (drx * dry - drw * drz);
			_r5 = 1.0 - 2.0 * (drx * drx + drz * drz);
			_r6 = 2.0 * (dry * drz + drw * drx);
			_r7 = 2.0 * (drx * drz + drw * dry);
			_r8 = 2.0 * (dry * drz - drw * drx);
			_r9 = 1.0 - 2.0 * (drx * drx + dry * dry);
		}

		public void ProcessMotion()
		{
			if (!enabled)
			{
				return;
			}

			//Compute local transformation
			double lpX, lpY, lpZ, lrX, lrY, lrZ, lrW;
			if(rotational)
			{
				ComputeLocalTransformation(BioIK.Deg2Rad*x.ProcessMotion(), BioIK.Deg2Rad*y.ProcessMotion(), BioIK.Deg2Rad*z.ProcessMotion(), out lpX, out lpY, out lpZ, out lrX, out lrY, out lrZ, out lrW);
			}
			else
			{
				ComputeLocalTransformation(x.ProcessMotion(), y.ProcessMotion(), z.ProcessMotion(), out lpX, out lpY, out lpZ, out lrX, out lrY, out lrZ, out lrW);
			}

			//Apply local transformation
			Transform t = transform;
			t.localPosition = new((float)lpX, (float)lpY, (float)lpZ);
			t.localRotation = new((float)lrX, (float)lrY, (float)lrZ, (float)lrW);

			//Remember transformation
			t.hasChanged = false;
		}

		//Fast implementation to compute the local transform given the joint values (in radians / metres)
		public void ComputeLocalTransformation(double valueX, double valueY, double valueZ, out double lpX, out double lpY, out double lpZ, out double lrX, out double lrY, out double lrZ, out double lrW)
		{
			if(!rotational)
			{
				Vector3 scale = transform.lossyScale;
				valueX /= scale.x;
				valueY /= scale.y;
				valueZ /= scale.z;
				double axisX = valueX * x.axis.x + valueY * y.axis.x + valueZ * z.axis.x;
				double axisY = valueX * x.axis.y + valueY * y.axis.y + valueZ * z.axis.y;
				double axisZ = valueX * x.axis.z + valueY * y.axis.z + valueZ * z.axis.z;
				//Local position for translational motion
				lpX = dpx + _r1 * axisX + _r4 * axisY + _r7 * axisZ;
				lpY = dpy + _r2 * axisX + _r5 * axisY + _r8 * axisZ;
				lpZ = dpz + _r3 * axisX + _r6 * axisY + _r9 * axisZ;
				//Local rotation for translational motion
				lrX = drx; lrY = dry; lrZ = drz; lrW = drw;
			}
			else
			{
				double sin, x1, y1, z1, w1, x2, y2, z2, w2, qx, qy, qz, qw;
				if (valueZ != 0.0)
				{
					sin = System.Math.Sin(valueZ / 2.0);
					qx = z.axis.x * sin;
					qy = z.axis.y * sin;
					qz = z.axis.z * sin;
					qw = System.Math.Cos(valueZ / 2.0);
					if (valueX != 0.0)
					{
						sin = System.Math.Sin(valueX / 2.0);
						x1 = x.axis.x * sin;
						y1 = x.axis.y * sin;
						z1 = x.axis.z * sin;
						w1 = System.Math.Cos(valueX / 2.0);
						x2 = qx; y2 = qy; z2 = qz; w2 = qw;
						qx = x1 * w2 + y1 * z2 - z1 * y2 + w1 * x2;
						qy = -x1 * z2 + y1 * w2 + z1 * x2 + w1 * y2;
						qz = x1 * y2 - y1 * x2 + z1 * w2 + w1 * z2;
						qw = -x1 * x2 - y1 * y2 - z1 * z2 + w1 * w2;
						if (valueY != 0.0)
						{
							sin = System.Math.Sin(valueY / 2.0);
							x1 = y.axis.x * sin;
							y1 = y.axis.y * sin;
							z1 = y.axis.z * sin;
							w1 = System.Math.Cos(valueY / 2.0);
							x2 = qx; y2 = qy; z2 = qz; w2 = qw;
							qx = x1 * w2 + y1 * z2 - z1 * y2 + w1 * x2;
							qy = -x1 * z2 + y1 * w2 + z1 * x2 + w1 * y2;
							qz = x1 * y2 - y1 * x2 + z1 * w2 + w1 * z2;
							qw = -x1 * x2 - y1 * y2 - z1 * z2 + w1 * w2;
						}
					}
					else if (valueY != 0.0)
					{
						sin = System.Math.Sin(valueY / 2.0);
						x1 = y.axis.x * sin;
						y1 = y.axis.y * sin;
						z1 = y.axis.z * sin;
						w1 = System.Math.Cos(valueY / 2.0);
						x2 = qx; y2 = qy; z2 = qz; w2 = qw;
						qx = x1 * w2 + y1 * z2 - z1 * y2 + w1 * x2;
						qy = -x1 * z2 + y1 * w2 + z1 * x2 + w1 * y2;
						qz = x1 * y2 - y1 * x2 + z1 * w2 + w1 * z2;
						qw = -x1 * x2 - y1 * y2 - z1 * z2 + w1 * w2;
					}
				}
				else if (valueX != 0.0)
				{
					sin = System.Math.Sin(valueX / 2.0);
					qx = x.axis.x * sin;
					qy = x.axis.y * sin;
					qz = x.axis.z * sin;
					qw = System.Math.Cos(valueX / 2.0);
					if (valueY != 0.0)
					{
						sin = System.Math.Sin(valueY / 2.0);
						x1 = y.axis.x * sin;
						y1 = y.axis.y * sin;
						z1 = y.axis.z * sin;
						w1 = System.Math.Cos(valueY / 2.0);
						x2 = qx; y2 = qy; z2 = qz; w2 = qw;
						qx = x1 * w2 + y1 * z2 - z1 * y2 + w1 * x2;
						qy = -x1 * z2 + y1 * w2 + z1 * x2 + w1 * y2;
						qz = x1 * y2 - y1 * x2 + z1 * w2 + w1 * z2;
						qw = -x1 * x2 - y1 * y2 - z1 * z2 + w1 * w2;
					}
				}
				else if(valueY != 0.0)
				{
					sin = System.Math.Sin(valueY / 2.0);
					qx = y.axis.x * sin;
					qy = y.axis.y * sin;
					qz = y.axis.z * sin;
					qw = System.Math.Cos(valueY / 2.0);
				}
				else
				{
					lpX = dpx;
					lpY = dpy;
					lpZ = dpz;
					lrX = drx;
					lrY = dry;
					lrZ = drz;
					lrW = drw;
					return;
				}

				//Local Rotation
				//R' = R*Q
				lrX = drx * qw + dry * qz - drz * qy + drw * qx;
				lrY = -drx * qz + dry * qw + drz * qx + drw * qy;
				lrZ = drx * qy - dry * qx + drz * qw + drw * qz;
				lrW = -drx * qx - dry * qy - drz * qz + drw * qw;

				//Local Position
				lpX = dpx;
				lpY = dpy;
				lpZ = dpz;
			}
		}

		public Vector3 GetAnchorInWorldSpace()
		{
			return transform.position;
		}

		public void SetOrientation(Vector3 value)
		{
			orientation = value;
			Quaternion o = Quaternion.Euler(orientation);
			x.axis = o * Vector3.right;
			y.axis = o * Vector3.up;
			z.axis = o * Vector3.forward;
		}

		public Vector3 GetOrientation()
		{
			return orientation;
		}

		public Vector3 GetDefaultPosition()
		{
			return new(dpx, dpy, dpz);
		}

		public Quaternion GetDefaultRotation()
		{
			return new(drx, dry, drz, drw);
		}

		public int GetDoF()
		{
			int dof = 0;
			if(x.IsEnabled())
			{
				dof += 1;
			}
			if(y.IsEnabled())
			{
				dof += 1;
			}
			if(z.IsEnabled())
			{
				dof += 1;
			}
			return dof;
		}

		private void SetDefaultFrame(Vector3 localPosition, Quaternion localRotation)
		{
			dpx = localPosition.x;
			dpy = localPosition.y;
			dpz = localPosition.z;
			drx = localRotation.x;
			dry = localRotation.y;
			drz = localRotation.z;
			drw = localRotation.w;
		}

		private void RestoreDefaultFrame()
		{
			Transform t = transform;
			t.localPosition = new(dpx, dpy, dpz);
			t.localRotation = new(drx, dry, drz, drw);
		}

		[System.Serializable]
		public class Motion
		{
			public BioJoint joint;

			public bool enabled;
			public double lowerLimit;
			public double upperLimit;
			public double targetValue;
			public Vector3 axis;

			public Motion(BioJoint joint, Vector3 axis)
			{
				this.joint = joint;
				this.axis = axis;
			}

			//Runs one motion control cycle
			public double ProcessMotion()
			{
				return !enabled ? 0.0 : targetValue;
			}

			public void SetEnabled(bool value)
			{
				if (enabled == value)
				{
					return;
				}

				enabled = value;
				joint.segment.controller.Refresh();
			}

			public bool IsEnabled()
			{
				return enabled;
			}

			public void SetLowerLimit(double value)
			{
				lowerLimit = System.Math.Min(0.0, value);
			}

			public double GetLowerLimit(bool normalised = false)
			{
				if (normalised && joint.rotational)
				{
					return BioIK.Deg2Rad * lowerLimit;
				}

				return lowerLimit;

			}

			public void SetUpperLimit(double value)
			{
				upperLimit = System.Math.Max(0.0, value);
			}

			public double GetUpperLimit(bool normalised = false)
			{
				if (normalised && joint.rotational)
				{
					return BioIK.Deg2Rad * upperLimit;
				}

				return upperLimit;

			}

			public void SetTargetValue(double value, bool normalised = false)
			{
				if (normalised && joint.rotational)
				{
					value *= BioIK.Rad2Deg;
				}

				targetValue = math.clamp(value, lowerLimit, upperLimit);
			}

			public double GetTargetValue(bool normalised = false)
			{
				if (normalised && joint.rotational)
				{
					return BioIK.Deg2Rad * targetValue;
				}

				return targetValue;
			}
		}
	}
}