using BioIK.Helpers;
using UnityEngine;

namespace BioIK.Setup {

	[AddComponentMenu("")]
	public class BioJoint : MonoBehaviour {
		public BioSegment segment;
		public Motion x,y,z;
		public JointType jointType = JointType.Rotational;							//Type of the joint

		[SerializeField] private Vector3 orientation = Vector3.zero;				//Joint orientation		
		[SerializeField] private float dpx, dpy, dpz, drx, dry, drz, drw;			//Default frame

		private Vector3 _animatedDefaultPosition;
		private Quaternion _animatedDefaultRotation;
		private double _r1, _r2, _r3, _r4, _r5, _r6, _r7, _r8, _r9;							//Precomputed rotation information
		private Vector3 _lsa;														//LocalScaledAnchor
		private Vector3 _adpadrsa;													//AnimatedDefaultPosition + AnimatedDefaultRotation * LocalScaledAnchor
		private Vector3 _scale;

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

		public BioJoint Create(BioSegment segment)
		{
			this.segment = segment;
			this.segment.transform.hideFlags = HideFlags.NotEditable;
			hideFlags = HideFlags.HideInInspector;

			x = new(this, Vector3.right);
			y = new(this, Vector3.up);
			z = new(this, Vector3.forward);

			Transform t = transform;
			SetDefaultFrame(t.localPosition, t.localRotation);

			Vector3 forward = Vector3.zero;
			if (this.segment.children.Length == 1)
			{
				forward = this.segment.children[0].transform.localPosition;
			}
			else if(this.segment.parent != null)
			{
				forward = Quaternion.Inverse(this.segment.transform.localRotation) * this.segment.transform.localPosition;
			}

			SetOrientation(forward.magnitude != 0f
				? Quaternion.LookRotation(forward, Vector3.up).eulerAngles
				: orientation);

			return this;
		}

		public void Remove()
		{
			RestoreDefaultFrame();
			segment.transform.hideFlags = HideFlags.None;
			if (segment != null)
			{
				segment.joint = null;
				if (segment.controller != null)
				{
					segment.controller.Refresh();
				}
			}
			Utility.Destroy(this);
		}

		public void Erase()
		{
			RestoreDefaultFrame();
			segment.transform.hideFlags = HideFlags.None;
			Utility.Destroy(this);
		}

		public void UpdateData() {
			_animatedDefaultPosition = new(dpx, dpy, dpz);
			_animatedDefaultRotation = new(drx, dry, drz, drw);
			_r1 = 1.0 - 2.0 * (_animatedDefaultRotation.y * _animatedDefaultRotation.y + _animatedDefaultRotation.z * _animatedDefaultRotation.z);
			_r2 = 2.0 * (_animatedDefaultRotation.x * _animatedDefaultRotation.y + _animatedDefaultRotation.w * _animatedDefaultRotation.z);
			_r3 = 2.0 * (_animatedDefaultRotation.x * _animatedDefaultRotation.z - _animatedDefaultRotation.w * _animatedDefaultRotation.y);
			_r4 = 2.0 * (_animatedDefaultRotation.x * _animatedDefaultRotation.y - _animatedDefaultRotation.w * _animatedDefaultRotation.z);
			_r5 = 1.0 - 2.0 * (_animatedDefaultRotation.x * _animatedDefaultRotation.x + _animatedDefaultRotation.z * _animatedDefaultRotation.z);
			_r6 = 2.0 * (_animatedDefaultRotation.y * _animatedDefaultRotation.z + _animatedDefaultRotation.w * _animatedDefaultRotation.x);
			_r7 = 2.0 * (_animatedDefaultRotation.x * _animatedDefaultRotation.z + _animatedDefaultRotation.w * _animatedDefaultRotation.y);
			_r8 = 2.0 * (_animatedDefaultRotation.y * _animatedDefaultRotation.z - _animatedDefaultRotation.w * _animatedDefaultRotation.x);
			_r9 = 1.0 - 2.0 * (_animatedDefaultRotation.x * _animatedDefaultRotation.x + _animatedDefaultRotation.y * _animatedDefaultRotation.y);
			_adpadrsa = _animatedDefaultPosition;
			_scale = transform.lossyScale;
		}

		public void ProcessMotion()
		{
			if (!enabled)
			{
				return;
			}

			//Compute local transformation
			double lpX, lpY, lpZ, lrX, lrY, lrZ, lrW;
			if(jointType == JointType.Rotational)
			{
				ComputeLocalTransformation(Utility.Deg2Rad*x.ProcessMotion(), Utility.Deg2Rad*y.ProcessMotion(), Utility.Deg2Rad*z.ProcessMotion(), out lpX, out lpY, out lpZ, out lrX, out lrY, out lrZ, out lrW);
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
			if(jointType == JointType.Translational)
			{
				valueX /= _scale.x;
				valueY /= _scale.y;
				valueZ /= _scale.z;
				double axisX = valueX * x.axis.x + valueY * y.axis.x + valueZ * z.axis.x;
				double axisY = valueX * x.axis.y + valueY * y.axis.y + valueZ * z.axis.y;
				double axisZ = valueX * x.axis.z + valueY * y.axis.z + valueZ * z.axis.z;
				//Local position for translational motion
				lpX = _animatedDefaultPosition.x + _r1 * axisX + _r4 * axisY + _r7 * axisZ;
				lpY = _animatedDefaultPosition.y + _r2 * axisX + _r5 * axisY + _r8 * axisZ;
				lpZ = _animatedDefaultPosition.z + _r3 * axisX + _r6 * axisY + _r9 * axisZ;
				//Local rotation for translational motion
				lrX = _animatedDefaultRotation.x; lrY = _animatedDefaultRotation.y; lrZ = _animatedDefaultRotation.z; lrW = _animatedDefaultRotation.w;
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
					lpX = _animatedDefaultPosition.x;
					lpY = _animatedDefaultPosition.y;
					lpZ = _animatedDefaultPosition.z;
					lrX = _animatedDefaultRotation.x;
					lrY = _animatedDefaultRotation.y;
					lrZ = _animatedDefaultRotation.z;
					lrW = _animatedDefaultRotation.w;
					return;
				}

				//Local Rotation
				//R' = R*Q
				lrX = _animatedDefaultRotation.x * qw + _animatedDefaultRotation.y * qz - _animatedDefaultRotation.z * qy + _animatedDefaultRotation.w * qx;
				lrY = -_animatedDefaultRotation.x * qz + _animatedDefaultRotation.y * qw + _animatedDefaultRotation.z * qx + _animatedDefaultRotation.w * qy;
				lrZ = _animatedDefaultRotation.x * qy - _animatedDefaultRotation.y * qx + _animatedDefaultRotation.z * qw + _animatedDefaultRotation.w * qz;
				lrW = -_animatedDefaultRotation.x * qx - _animatedDefaultRotation.y * qy - _animatedDefaultRotation.z * qz + _animatedDefaultRotation.w * qw;

				//Local Position
				if (_lsa.x == 0.0 && _lsa.y == 0.0 && _lsa.z == 0.0)
				{
					//P' = Pz
					lpX = _animatedDefaultPosition.x;
					lpY = _animatedDefaultPosition.y;
					lpZ = _animatedDefaultPosition.z;
				}
				else
				{
					//P' = P + RA + R*Q*(-A)
					lpX = _adpadrsa.x + 2.0 * ((0.5 - lrY * lrY - lrZ * lrZ) * -_lsa.x + (lrX * lrY - lrW * lrZ) * -_lsa.y + (lrX * lrZ + lrW * lrY) * -_lsa.z);
					lpY = _adpadrsa.y + 2.0 * ((lrX * lrY + lrW * lrZ) * -_lsa.x + (0.5 - lrX * lrX - lrZ * lrZ) * -_lsa.y + (lrY * lrZ - lrW * lrX) * -_lsa.z);
					lpZ = _adpadrsa.z + 2.0 * ((lrX * lrZ - lrW * lrY) * -_lsa.x + (lrY * lrZ + lrW * lrX) * -_lsa.y + (0.5 - lrX * lrX - lrY * lrY) * -_lsa.z);
				}
			}
		}

		public Vector3 GetAnchorInWorldSpace()
		{
			return transform.position;
		}

		public void SetOrientation(Vector3 orientation)
		{
			this.orientation = orientation;
			Quaternion o = Quaternion.Euler(this.orientation);
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

		public void SetDefaultFrame(Vector3 localPosition, Quaternion localRotation)
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
			public bool constrained = true;
			public double lowerLimit;
			public double upperLimit;
			public double targetValue;
			public double currentValue;
			public double currentError;
			public Vector3 axis;

			public Motion(BioJoint joint, Vector3 axis)
			{
				this.joint = joint;
				this.axis = axis;
			}

			//Runs one motion control cycle
			public double ProcessMotion()
			{
				if (!enabled)
				{
					return 0.0;
				}

				currentValue = targetValue;
				currentError = 0f;

				return currentValue;
			}

			public void SetEnabled(bool enabled)
			{
				if (this.enabled == enabled)
				{
					return;
				}

				this.enabled = enabled;
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
				if (!constrained)
				{
					return double.MinValue;
				}

				if (normalised && joint.jointType == JointType.Rotational)
				{
					return Utility.Deg2Rad * lowerLimit;
				}

				return lowerLimit;

			}

			public void SetUpperLimit(double value)
			{
				upperLimit = System.Math.Max(0.0, value);
			}

			public double GetUpperLimit(bool normalised = false)
			{
				if (!constrained)
				{
					return double.MaxValue;
				}

				if (normalised && joint.jointType == JointType.Rotational)
				{
					return Utility.Deg2Rad * upperLimit;
				}

				return upperLimit;

			}

			public void SetTargetValue(double value, bool normalised = false)
			{
				if (normalised && joint.jointType == JointType.Rotational)
				{
					value *= Utility.Rad2Deg;
				}
				if (constrained)
				{
					if (targetValue > upperLimit)
					{
						value = upperLimit;
					}
					if(targetValue < lowerLimit)
					{
						value = lowerLimit;
					}
				}
				targetValue = value;
			}

			public double GetTargetValue(bool normalised = false)
			{
				if (normalised && joint.jointType == JointType.Rotational)
				{
					return Utility.Deg2Rad * targetValue;
				}

				return targetValue;
			}

			public double GetCurrentValue()
			{
				return currentValue;
			}

			public double GetCurrentError()
			{
				return currentError;
			}
		}
	}
}