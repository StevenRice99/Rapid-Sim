using UnityEngine;
using System.Collections.Generic;
using BioIK.Helpers;
using BioIK.Setup;
using Unity.Mathematics;

namespace BioIK
{
	[DisallowMultipleComponent]
	public class BioRobot : MonoBehaviour
	{
		public const double Deg2Rad = 0.017453292;
		public const double Rad2Deg = 57.29578049;
		public const double PI = 3.14159265358979;
		
		public List<BioSegment> segments = new();
		public BioSegment root;
		public Evolution evolution;
		public double[] solution;
		public int generations = 2;
		
		private int _populationSize = 50;
		private int _elites = 2;
		private bool _destroyed;

		private void Awake()
		{
			Refresh();
		}

		private void OnDestroy()
		{
			_destroyed = true;
			DeInitialise();
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
			evolution ??= new(new(this), _populationSize, _elites);
		}

		public void DeInitialise()
		{
			evolution = null;
		}

		public void SetPopulationSize(int size)
		{
			if (_populationSize == size)
			{
				return;
			}

			_populationSize = math.max(1, size);
			_elites = math.min(size, _elites);
			if (Application.isPlaying)
			{
				Refresh();
			}
		}

		public void SetElites(int number)
		{
			if (_elites == number)
			{
				return;
			}

			_elites = math.max(1, number);
			if (Application.isPlaying)
			{
				Refresh();
			}
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
			while (segment != null)
			{
				if (segment.joint != null)
				{
					segment.joint.UpdateData();
				}
			
				if (segment.objective != null)
				{
					segment.objective.UpdateData();
				}

				segment = segment.child;
			}
		}

		public void Refresh(bool evolve = true)
		{
			if (_destroyed)
			{
				return;
			}

			segments = new();

			Transform t = transform;
			while (t != null)
			{
				BioSegment segment = t.GetComponent<BioSegment>();
				if (segment == null)
				{
					segment = t.gameObject.AddComponent(typeof(BioSegment)) as BioSegment;
					segment.bioRobot = this;
					segments.Add(segment);
				}
			
				segment.bioRobot = this;
				
				if (segment.transform.parent != null)
				{
					segment.parent = segment.transform.parent.GetComponent<BioSegment>();
					if (segment.parent != null)
					{
						segment.parent.child = segment;
					}
				}
				else
				{
					segment.parent = null;
				}

				if (t.childCount == 0)
				{
					break;
				}

				t = t.GetChild(0);
			}
			
			root = GetComponent<BioSegment>();

			if (!evolve || !Application.isPlaying)
			{
				return;
			}

			DeInitialise();
			Initialise();
			solution = new double[evolution.GetModel().GetDoF()];
		}

		public static void ProcessMotion(BioSegment segment)
		{
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