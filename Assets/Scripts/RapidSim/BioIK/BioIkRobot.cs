using UnityEngine;

namespace RapidSim.BioIK
{
	[DisallowMultipleComponent]
	public class BioIkRobot : MonoBehaviour
	{
		[Tooltip("The root of the Bio IK chain.")]
		public BioIkSegment root;

		[Tooltip("The best solution from the Bio IK algorithm.")]
		public double[] solution;
		
		public BioIkEvolution Evolution { get; private set; }

		public void Initialise(int populationSize, int elites, double rescaling)
		{
			Evolution = new(new(this, rescaling), populationSize, elites, rescaling);
		}

		public void UpdateData()
		{
			BioIkSegment segment = root;
			
			while (segment != null)
			{
				if (segment.joint != null)
				{
					segment.joint.UpdateData();
				}

				segment = segment.child;
			}
		}

		public void ProcessMotion()
		{
			BioIkSegment segment = root;
			
			while (segment != null)
			{
				if (segment.joint != null)
				{
					segment.joint.ProcessMotion();
				}

				segment = segment.child;
			}
		}
	}
}