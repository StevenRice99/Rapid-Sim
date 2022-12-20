using UnityEngine;

namespace BioIK.Setup
{
	[DisallowMultipleComponent]
	public class BioSegment : MonoBehaviour
	{
		public BioRobot bioRobot;
		public BioSegment parent;
		public BioSegment child;
		public BioJoint joint;
		public BioObjective objective;
	}
}