using BioIK.Helpers;
using UnityEngine;

namespace BioIK.Setup
{
	[AddComponentMenu("")]
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
				segment.controller.Refresh();
			}
		}

		private void OnDisable()
		{
			if (segment != null)
			{
				segment.controller.Refresh();
			}
		}

		public BioObjective Create(BioSegment value)
		{
			segment = value;
			hideFlags = HideFlags.HideInInspector;
			return this;
		}

		public void Remove()
		{
			for (int i = 0; i < segment.objectives.Length; i++)
			{
				if (segment.objectives[i] != this)
				{
					continue;
				}

				for (int j = i; j < segment.objectives.Length - 1; j++)
				{
					segment.objectives[j] = segment.objectives[j+1];
				}
				
				System.Array.Resize(ref segment.objectives, segment.objectives.Length-1);
				break;
			}
			
			if (segment != null && segment.controller != null)
			{
				segment.controller.Refresh();
			}
			
			Utility.Destroy(this);
		}

		public void Erase()
		{
			Utility.Destroy(this);
		}

		public void UpdateData()
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

		public Vector3 GetTargetPosition()
		{
			return new((float)_tpx, (float)_tpy, (float)_tpz);
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