using UnityEngine;
using System.Collections.Generic;
using BioIK.Helpers;
using BioIK.Setup;
using Unity.Mathematics;

namespace BioIK
{
	[DisallowMultipleComponent]
	public class BioIK : MonoBehaviour
	{
		public int generations = 2;
		
		public int PopulationSize { get; private set; } = 50;
		
		public int Elites { get; private set; } = 2;

		public List<BioSegment> segments = new();

		public BioSegment root;
		public Evolution Evolution;
		public double[] solution;

		private bool _destroyed;

		//Custom Inspector Helpers
		public BioSegment selectedSegment;
		public Vector2 scroll = Vector2.zero;

		private void Awake()
		{
			Refresh();
		}

		private void OnDestroy()
		{
			_destroyed = true;
			DeInitialise();
			Utility.Cleanup(transform);
		}

		private void OnEnable()
		{
			Initialise();	
		}

		private void OnDisable()
		{
			DeInitialise();
		}

		public void Initialise()
		{
			Evolution ??= new(new(this), PopulationSize, Elites);
		}

		public void DeInitialise()
		{
			if (Evolution == null)
			{
				return;
			}

			Evolution.Kill();
			Evolution = null;
		}

		public void SetPopulationSize(int size)
		{
			if (PopulationSize == size)
			{
				return;
			}

			PopulationSize = math.max(1, size);
			Elites = math.min(size, Elites);
			if (Application.isPlaying)
			{
				Refresh();
			}
		}

		public void SetElites(int number)
		{
			if (Elites == number)
			{
				return;
			}

			Elites = math.max(1, number);
			if (Application.isPlaying)
			{
				Refresh();
			}
		}

		public BioSegment FindSegment(Transform t)
		{
			for (int i = 0; i < segments.Count; i++)
			{
				if (segments[i].transform == t)
				{
					return segments[i];
				}
			}
			return null;
		}

		public List<BioSegment> GetChain(BioSegment end)
		{
			List<BioSegment> chain = new();
			BioSegment segment = end;
			while(true)
			{
				chain.Add(segment);
				if(segment.transform == transform || segment.parent == null)
				{
					break;
				}

				segment = segment.parent;
			}
			chain.Reverse();
			return chain;
		}

		public static void UpdateData(BioSegment segment)
		{
			if (segment.joint != null && segment.joint.enabled)
			{
				segment.joint.UpdateData();
			}
			
			for (int i = 0; i < segment.objectives.Length; i++)
			{
				if (segment.objectives[i].enabled)
				{
					segment.objectives[i].UpdateData();
				}
			}
			
			for (int i = 0; i<segment.children.Length; i++)
			{
				UpdateData(segment.children[i]);
			}
		}

		public void Refresh(bool evolution = true)
		{
			if (_destroyed)
			{
				return;
			}
			
			for (int i = 0; i < segments.Count; i++)
			{
				if (segments[i] != null)
				{
					continue;
				}

				segments.RemoveAt(i);
				i--;
			}
			
			Refresh(transform);
			root = FindSegment(transform);

			if (!evolution || !Application.isPlaying)
			{
				return;
			}

			DeInitialise();
			Initialise();
			solution = new double[Evolution.GetModel().GetDoF()];
		}

		private void Refresh(Transform t)
		{
			BioSegment segment = FindSegment(t);
			if (segment == null)
			{
				segment = Utility.AddBioSegment(this, t);
				segments.Add(segment);
			}
			
			segment.controller = this;
			segment.RenewRelations();
			
			for (int i = 0; i < t.childCount; i++)
			{
				Refresh(t.GetChild(i));
			}
		}

		public static void ProcessMotion(BioSegment segment)
		{
			if (segment.joint != null && segment.joint.enabled)
			{
				segment.joint.ProcessMotion();
			}
			
			for (int i = 0; i < segment.children.Length; i++)
			{
				ProcessMotion(segment.children[i]);
			}
		}

	}
}