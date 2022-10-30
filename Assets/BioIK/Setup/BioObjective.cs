using BioIK.Helpers;
using UnityEngine;

namespace BioIK.Setup {

	[AddComponentMenu("")]
	public abstract class BioObjective : MonoBehaviour {
		public BioSegment Segment;
		public double Weight = 1.0;

		private void Awake() {

		}

		private void Start() {

		}

		private void OnEnable() {
			if(Segment != null) {
				Segment.controller.Refresh();
			}
		}

		private void OnDisable() {
			if(Segment != null) {
				Segment.controller.Refresh();
			}
		}

		private void OnDestroy() {

		}

		public BioObjective Create(BioSegment segment) {
			Segment = segment;
			hideFlags = HideFlags.HideInInspector;
			return this;
		}

		public void Remove() {
			for(int i=0; i<Segment.objectives.Length; i++) {
				if(Segment.objectives[i] == this) {
					for(int j=i; j<Segment.objectives.Length-1; j++) {
						Segment.objectives[j] = Segment.objectives[j+1];
					}
					System.Array.Resize(ref Segment.objectives, Segment.objectives.Length-1);
					break;
				}
			}
			if(Segment != null) {
				if(Segment.controller != null) {
					Segment.controller.Refresh();
				}
			}
			Utility.Destroy(this);
		}

		public void Erase() {
			Utility.Destroy(this);
		}

		public void SetWeight(double weight) {
			if(weight < 0.0) {
				Debug.Log("Weight must be at least zero.");
				Weight = 0.0;
				return;
			}
			Weight = weight;
		}

		public double GetWeight() {
			return Weight;
		}

		public abstract ObjectiveType GetObjectiveType();
		public abstract void UpdateData();
		public abstract double ComputeLoss(double WPX, double WPY, double WPZ, double WRX, double WRY, double WRZ, double WRW, Model.Node node, double[] configuration);
		public abstract bool CheckConvergence(double WPX, double WPY, double WPZ, double WRX, double WRY, double WRZ, double WRW, Model.Node node, double[] configuration);
		public abstract double ComputeValue(double WPX, double WPY, double WPZ, double WRX, double WRY, double WRZ, double WRW, Model.Node node, double[] configuration);
	}

}