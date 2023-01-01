using UnityEngine;

namespace RapidSim.BioIK
{
	[DisallowMultipleComponent]
	public class BioIkSegment : MonoBehaviour
	{
		[Tooltip("The parent segment.")]
		public BioIkSegment parent;
		
		[Tooltip("The child segment.")]
		public BioIkSegment child;
		
		[Tooltip("The joint attached to this segment.")]
		public BioIkJoint joint;
	}
}