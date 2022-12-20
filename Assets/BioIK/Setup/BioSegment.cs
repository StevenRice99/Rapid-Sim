using System;
using BioIK.Helpers;
using UnityEngine;

namespace BioIK.Setup
{
	[AddComponentMenu("")]
	public class BioSegment : MonoBehaviour
	{
		public BioIK controller;
		public BioSegment parent;
		public BioSegment[] children = Array.Empty<BioSegment>();
		public BioJoint joint;
		public BioObjective[] objectives = Array.Empty<BioObjective>();

		public BioSegment Create(BioIK bioIk)
		{
			controller = bioIk;
			hideFlags = HideFlags.HideInInspector;
			return this;
		}

		public void AddChild(BioSegment child)
		{
			Array.Resize(ref children, children.Length+1);
			children[^1] = child;
		}

		public void RenewRelations()
		{
			parent = null;
			Array.Resize(ref children, 0);
			if (transform == controller.transform)
			{
				return;
			}

			parent = controller.FindSegment(transform.parent);
			parent.AddChild(this);
		}

		public BioJoint AddJoint()
		{
			if(joint != null)
			{
				Debug.Log("The segment already has a joint.");
			}
			else
			{
				joint = (gameObject.AddComponent(typeof(BioJoint)) as BioJoint)?.Create(this);
				controller.Refresh();
			}
			return joint;
		}

		public BioObjective AddObjective()
		{
			BioObjective objective = (gameObject.AddComponent(typeof(BioObjective)) as BioObjective)?.Create(this);
			Array.Resize(ref objectives, objectives.Length+1);
			objectives[^1] = objective;
			controller.Refresh();
			return objectives[^1];
		}
	}
}