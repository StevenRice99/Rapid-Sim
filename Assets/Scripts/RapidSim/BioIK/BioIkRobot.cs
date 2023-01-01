using UnityEngine;

namespace RapidSim.BioIK
{
	[DisallowMultipleComponent]
	public class BioIkRobot : MonoBehaviour
	{
		public const double Deg2Rad = 0.017453292;
		public const double Rad2Deg = 57.29578049;
		public const double PI = 3.14159265358979;
		
		[Tooltip("The root of the Bio IK chain.")]
		public BioIkSegment root;

		[Tooltip("The best solution from the Bio IK algorithm.")]
		public double[] solution;
		
		public BioIkEvolution Evolution { get; private set; }

		public void Initialise(int populationSize, int elites)
		{
			Evolution = new(new(this), populationSize, elites);
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