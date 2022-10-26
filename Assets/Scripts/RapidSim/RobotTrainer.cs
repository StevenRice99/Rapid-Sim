using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Random = UnityEngine.Random;

namespace RapidSim
{
    [DisallowMultipleComponent]
    [RequireComponent(typeof(RobotSolver))]
    public class RobotTrainer : MonoBehaviour
    {
        [Min(1)]
        [SerializeField]
        private int maxSteps = 1;
        
        public RobotController RobotController { get; private set; }

        public RobotSolver RobotSolver { get; private set; }

        private BioIK.BioIK _bioIK;

        public List<float> Joints => RobotController.GetJoints();

        public Transform Objective => RobotController.LastJoint.transform;
    
        private void Start()
        {
            RobotController = GetComponent<RobotController>();
            RobotSolver = GetComponent<RobotSolver>();
        }

        private void OnEnable()
        {
            RobotTrainerManager.Register(this);
            SetupBioIk();
        }

        private void OnDisable()
        {
            RobotTrainerManager.Unregister(this);
            Destroy(_bioIK.gameObject);
        }

        public void Train(Vector3 position, Quaternion rotation, List<float> joints, float[] expected)
        {
            RobotSolver.Net.Train(RobotSolver.PrepareInputs(RobotSolver.NetScaled(joints), position, rotation), RobotSolver.NetScaled(expected.ToList()).ToArray());
        }

        public List<float> RandomOrientation()
        {
            List<float> joints = new();
            for (int i = 0; i < RobotController.LowerLimits.Length; i++)
            {
                joints.Add(Random.Range(RobotController.LowerLimits[i], RobotController.UpperLimits[i]));
            }

            return joints;
        }

        public void SetRandomOrientation()
        {
            RobotController.SnapRadians(RandomOrientation());
        }

        private void LateUpdate()
        {
            if (RobotSolver.NetworkSteps >= maxSteps)
            {
                Destroy(this);
            }
        }

        private void SetupBioIk()
        {
            Transform rootTransform = RobotController.Root.transform;
            GameObject bioIkHolder = new("Bio IK")
            {
                transform =
                {
                    parent = transform,
                    position = rootTransform.position,
                    rotation = rootTransform.rotation
                }
            };
            _bioIK = bioIkHolder.AddComponent<BioIK.BioIK>();
        }
    }
}