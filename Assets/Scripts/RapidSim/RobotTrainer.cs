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

        private BioJoint[] _bioJoints;

        private Position _position;

        private Orientation _orientation;

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

        public float[] BioIkSolve(Vector3 position, Quaternion rotation)
        {
            _position.SetTargetPosition(position);
            _orientation.SetTargetRotation(rotation);
            
            _bioIK.UpdateData(_bioIK.Root);
            
            for(int i = 0; i < _bioIK.Solution.Length; i++) {
                _bioIK.Solution[i] = _bioIK.Evolution.GetModel().MotionPtrs[i].Motion.GetTargetValue(true);
            }
            
            _bioIK.Solution = _bioIK.Evolution.Optimise(_bioIK.GetGenerations(), _bioIK.Solution);

            for(int i = 0; i< _bioIK.Solution.Length; i++) {
                BioJoint.Motion motion = _bioIK.Evolution.GetModel().MotionPtrs[i].Motion;
                motion.SetTargetValue(_bioIK.Solution[i], true);
            }

            _bioIK.ProcessMotion(_bioIK.Root);

            List<float> expected = new();
            for (int i = 0; i < _bioJoints.Length; i++)
            {
                if (_bioJoints[i].X.Enabled)
                {
                    float value = (float) _bioJoints[i].X.CurrentValue;
                    if (_bioJoints[i].JointType == JointType.Rotational)
                    {
                        value *= Mathf.Deg2Rad;
                    }
                    expected.Add(value);
                }
                
                if (_bioJoints[i].Y.Enabled)
                {
                    float value = (float) _bioJoints[i].Y.CurrentValue;
                    if (_bioJoints[i].JointType == JointType.Rotational)
                    {
                        value *= Mathf.Deg2Rad;
                    }
                    expected.Add(value);
                }
                
                if (_bioJoints[i].Z.Enabled)
                {
                    float value = (float) _bioJoints[i].Y.CurrentValue;
                    if (_bioJoints[i].JointType == JointType.Rotational)
                    {
                        value *= Mathf.Deg2Rad;
                    }
                    expected.Add(value);
                }
            }

            return RobotSolver.NetScaled(expected).ToArray();
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

            List<BioJoint> bioJoints = new();

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

                if (i == RobotController.Joints.Length - 1)
                {
                    _position = segment.AddObjective(ObjectiveType.Position) as Position;
                    if (_position != null)
                    {
                        _position.SetMaximumError(0);
                    }

                    _orientation = segment.AddObjective(ObjectiveType.Orientation) as Orientation;
                    if (_orientation != null)
                    {
                        _orientation.SetMaximumError(0);
                    }
                }

                if (!RobotController.Joints[i].HasMotion)
                {
                    continue;
                }

                BioJoint bioJoint = segment.AddJoint();
                bioJoint.JointType = RobotController.Joints[i].Type == ArticulationJointType.PrismaticJoint ? JointType.Translational : JointType.Rotational;
                bioJoint.SetOrientation(Vector3.zero);
                bioJoint.SetAnchor(Vector3.zero);
                bioJoints.Add(bioJoint);

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

            _bioJoints = bioJoints.ToArray();
        }
    }
}