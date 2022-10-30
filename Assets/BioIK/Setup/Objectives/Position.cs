using BioIK.Helpers;
using UnityEngine;

namespace BioIK.Setup.Objectives
{
	//This objective aims to minimise the translational distance between the transform and the target.
	[AddComponentMenu("")]
	public class Position : BioObjective
	{
		private double _tpx, _tpy, _tpz;

		private double _chainLength;
		private double _rescaling;

		public override ObjectiveType GetObjectiveType()
		{
			return ObjectiveType.Position;
		}

		public override void UpdateData()
		{
			if (segment.controller.Evolution == null)
			{
				return;
			}
			
			_chainLength = 0.0;
			Transform[] chain = segment.controller.Evolution.GetModel().FindObjectivePtr(this).Node.Chain;
			for(int i = 0; i < chain.Length-1; i++)
			{
				_chainLength += Vector3.Distance(chain[i].position, chain[i+1].position);
			}
			_rescaling = Utility.PI * Utility.PI / (_chainLength * _chainLength);
		}

		public override double ComputeLoss(double wpx, double wpy, double wpz, double wrx, double wry, double wrz, double wrw)
		{
			return _rescaling * ((_tpx-wpx)*(_tpx-wpx) + (_tpy-wpy)*(_tpy-wpy) + (_tpz-wpz)*(_tpz-wpz));
		}

		public void SetTargetPosition(Vector3 position)
		{
			_tpx = position.x;
			_tpy = position.y;
			_tpz = position.z;
		}

		public Vector3 GetTargetPosition()
		{
			return new((float)_tpx, (float)_tpy, (float)_tpz);
		}
	}
}