using System.Collections.Generic;
using System.Linq;
using BioIK;
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
            SetupBioIk();
        }

        private void OnEnable()
        {
            RobotTrainerManager.Register(this);
        }

        private void OnDisable()
        {
            RobotTrainerManager.Unregister(this);
        }

        private void OnDestroy()
        {
            if (_bioIK == null)
            {
                return;
            }
            
            Destroy(_bioIK.gameObject);
        }

        public void Train(Vector3 position, Quaternion rotation, List<float> joints, float[] expected)
        {
            RobotSolver.Net.Train(RobotSolver.PrepareInputs(RobotSolver.NetScaled(joints), position, rotation), RobotSolver.NetScaled(expected.ToList()).ToArray());
        }

        public List<float> RandomOrientation()
        {
            List<float> joints = new();
            for (int i = 0; i < RobotController.Limits.Length; i++)
            {
                joints.Add(Random.Range(RobotController.Limits[i].Lower, RobotController.Limits[i].Upper));
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
            _bioIK.SetThreading(false);
            _bioIK.Smoothing = 0;

            Transform parent = _bioIK.transform;
            for (int i = 0; i < RobotController.Joints.Length; i++)
            {
                GameObject go = new($"Joint {i}")
                {
                    transform =
                    {
                        parent = parent,
                        position = RobotController.Joints[i].transform.position,
                        rotation = RobotController.Joints[i].transform.rotation
                    }
                };

                _bioIK.Refresh(false);
                BioSegment segment = go.GetComponent<BioSegment>();
                parent = go.transform;

                if (!RobotController.Joints[i].HasMotion)
                {
                    continue;
                }

                BioJoint bioJoint = segment.AddJoint();
                bioJoint.JointType = RobotController.Joints[i].Type == ArticulationJointType.PrismaticJoint ? JointType.Translational : JointType.Rotational;
                bioJoint.SetOrientation(Vector3.zero);
                bioJoint.SetAnchor(Vector3.zero);

                bioJoint.X = new(bioJoint, Vector3.zero)
                {
                    Constrained = true
                };
                if (RobotController.Joints[i].XMotion)
                {
                    bioJoint.X.Enabled = true;
                    if (bioJoint.JointType == JointType.Translational)
                    {
                        bioJoint.X.LowerLimit = RobotController.Joints[i].LimitX.Lower;
                        bioJoint.X.UpperLimit = RobotController.Joints[i].LimitX.Upper;
                    }
                    else
                    {
                        bioJoint.X.LowerLimit = RobotController.Joints[i].LimitX.Lower * Mathf.Rad2Deg;
                        bioJoint.X.UpperLimit = RobotController.Joints[i].LimitX.Upper * Mathf.Rad2Deg;
                    }
                }
                else
                {
                    bioJoint.X.Enabled = false;
                }

                bioJoint.Y = new(bioJoint, Vector3.zero)
                {
                    Constrained = true
                };
                if (RobotController.Joints[i].YMotion)
                {
                    bioJoint.Y.Enabled = true;
                    if (bioJoint.JointType == JointType.Translational)
                    {
                        bioJoint.Y.LowerLimit = RobotController.Joints[i].LimitY.Lower;
                        bioJoint.Y.UpperLimit = RobotController.Joints[i].LimitY.Upper;
                    }
                    else
                    {
                        bioJoint.Y.LowerLimit = RobotController.Joints[i].LimitY.Lower * Mathf.Rad2Deg;
                        bioJoint.Y.UpperLimit = RobotController.Joints[i].LimitY.Upper * Mathf.Rad2Deg;
                    }
                }
                else
                {
                    bioJoint.Y.Enabled = false;
                }
                
                bioJoint.Z = new(bioJoint, Vector3.zero)
                {
                    Constrained = true
                };
                if (RobotController.Joints[i].ZMotion)
                {
                    bioJoint.Z.Enabled = true;
                    if (bioJoint.JointType == JointType.Translational)
                    {
                        bioJoint.Z.LowerLimit = RobotController.Joints[i].LimitZ.Lower;
                        bioJoint.Z.UpperLimit = RobotController.Joints[i].LimitZ.Upper;
                    }
                    else
                    {
                        bioJoint.Z.LowerLimit = RobotController.Joints[i].LimitZ.Lower * Mathf.Rad2Deg;
                        bioJoint.Z.UpperLimit = RobotController.Joints[i].LimitZ.Upper * Mathf.Rad2Deg;
                    }
                }
                else
                {
                    bioJoint.Z.Enabled = false;
                }
            }
        }
    }
}