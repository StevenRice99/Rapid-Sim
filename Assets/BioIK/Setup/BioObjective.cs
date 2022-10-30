using BioIK.Helpers;
using UnityEngine;

namespace BioIK.Setup
{
	[AddComponentMenu("")]
	public abstract class BioObjective : MonoBehaviour
	{
		public BioSegment segment;

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

		public abstract ObjectiveType GetObjectiveType();

		public virtual void UpdateData() { }

		public abstract double ComputeLoss(double wpx, double wpy, double wpz, double wrx, double wry, double wrz, double wrw);
	}
}