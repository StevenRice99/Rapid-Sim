using UnityEngine;

namespace BioIK.Setup
{
	[DisallowMultipleComponent]
	public class BioObjective : MonoBehaviour
	{
		public BioSegment segment;
		
		private double _tpx, _tpy, _tpz;

		private double _chainLength;
		private double _rescaling;
		
		private double _trx, _try, _trz, _trw;

		private void OnEnable()
		{
			if (segment != null)
			{
				segment.bioRobot.Refresh();
			}
		}

		private void OnDisable()
		{
			if (segment != null)
			{
				segment.bioRobot.Refresh();
			}
		}

		public void UpdateData()
		{
			if (segment.bioRobot.evolution == null)
			{
				return;
			}
			
			_chainLength = 0.0;
			Transform[] chain = segment.bioRobot.evolution.GetModel().FindObjectivePtr(this).node.chain;
			for(int i = 0; i < chain.Length-1; i++)
			{
				_chainLength += Vector3.Distance(chain[i].position, chain[i+1].position);
			}
			_rescaling = BioRobot.PI * BioRobot.PI / (_chainLength * _chainLength);
		}

		public double ComputeLoss(double wpx, double wpy, double wpz, double wrx, double wry, double wrz, double wrw)
		{
			double pos = _rescaling * ((_tpx-wpx)*(_tpx-wpx) + (_tpy-wpy)*(_tpy-wpy) + (_tpz-wpz)*(_tpz-wpz));
			
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
			double rot = 2.0 * System.Math.Acos(d);
			rot *= rot;
			
			return pos + rot;
		}
		
		public void SetTargetPosition(Vector3 position)
		{
			_tpx = position.x;
			_tpy = position.y;
			_tpz = position.z;
		}
		
		public void SetTargetRotation(Quaternion rotation)
		{
			_trx = rotation.x;
			_try = rotation.y;
			_trz = rotation.z;
			_trw = rotation.w;
		}
	}
}