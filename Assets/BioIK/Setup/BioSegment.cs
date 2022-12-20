using UnityEngine;

namespace BioIK.Setup
{
	[AddComponentMenu("")]
	public class BioSegment : MonoBehaviour
	{
		public BioIK controller;
		public BioSegment parent;
		public BioSegment child;
		public BioJoint joint;
		public BioObjective objective;

		public BioSegment Create(BioIK bioIk)
		{
			controller = bioIk;
			hideFlags = HideFlags.HideInInspector;
			return this;
		}

		public void RenewRelations()
		{
			parent = null;
			if (transform == controller.transform)
			{
				return;
			}

			parent = controller.FindSegment(transform.parent);
			parent.child = this;
		}

		public BioJoint AddJoint()
		{
			joint = (gameObject.AddComponent(typeof(BioJoint)) as BioJoint)?.Create(this);
			controller.Refresh();
			return joint;
		}

		public BioObjective AddObjective()
		{
			objective = (gameObject.AddComponent(typeof(BioObjective)) as BioObjective)?.Create(this);
			return objective;
		}
	}
}