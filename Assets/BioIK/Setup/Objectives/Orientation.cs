using BioIK.Helpers;
using UnityEngine;

namespace BioIK.Setup.Objectives
{
	//This objective aims to minimise the rotational distance between the transform and the target.
	[AddComponentMenu("")]
	public class Orientation : BioObjective
	{
		private double _trx, _try, _trz, _trw;

		public override ObjectiveType GetObjectiveType()
		{
			return ObjectiveType.Orientation;
		}

		public override double ComputeLoss(double wpx, double wpy, double wpz, double wrx, double wry, double wrz, double wrw)
		{
			double d = wrx*_trx + wry*_try + wrz*_trz + wrw*_trw;
			switch (d)
			{
				case < 0.0:
				{
					d = -d;
					if(d > 1.0) {
						d = 1.0;
					}

					break;
				}
				case > 1.0:
					d = 1.0;
					break;
			}
			double loss = 2.0 * System.Math.Acos(d);
			return loss * loss;
		}

		public void SetTargetRotation(Quaternion rotation)
		{
			_trx = rotation.x;
			_try = rotation.y;
			_trz = rotation.z;
			_trw = rotation.w;
		}

		public void SetTargetRotation(Vector3 angles)
		{
			SetTargetRotation(Quaternion.Euler(angles));
		}

		public Vector3 GetTargetRotation()
		{
			return new Quaternion((float)_trx, (float)_try, (float)_trz, (float)_trw).eulerAngles;
		}
	}
}