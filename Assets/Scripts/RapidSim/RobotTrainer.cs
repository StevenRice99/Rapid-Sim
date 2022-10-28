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
        
        [SerializeField]
        private bool train;
        
        public RobotController RobotController { get; private set; }

        public RobotSolver RobotSolver { get; private set; }

        private BioIK.BioIK _bioIK;

        private BioJoint[] _bioJoints;

        private Position _position;

        private Orientation _orientation;

        private Transform _bioIkTarget;

        private bool _train;

        public Transform Objective => RobotController.LastJoint.transform;
    
        private void Start()
        {
            RobotController = GetComponent<RobotController>();
            RobotSolver = GetComponent<RobotSolver>();
            SetupBioIk();
        }

        private void OnDestroy()
        {
            if (_bioIK == null)
            {
                return;
            }
            
            Destroy(_bioIK.gameObject);
        }

        private void OnEnable()
        {
            _train = train;
            UpdateTraining();
        }

        public float[] BioIkSolve(Vector3 position, Quaternion orientation)
        {
            _bioIkTarget.position = position;
            _bioIkTarget.rotation = orientation;
            
            _bioIK.Refresh();
            
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

        private void Update()
        {
            if (RobotSolver.NetworkSteps >= maxSteps)
            {
                Destroy(this);
            }

            if (train == _train)
            {
                return;
            }

            _train = train;
            UpdateTraining();
        }

        private void UpdateTraining()
        {
            if (train)
            {
                RobotTrainerManager.Register(this);
            }
            else
            {
                RobotTrainerManager.Unregister(this);
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
            
            List<BioSegment> segments = new();

            _bioIK.Refresh(false);
            BioSegment rootSegment = bioIkHolder.GetComponent<BioSegment>();
            rootSegment = rootSegment.Create(_bioIK);
            rootSegment.RenewRelations();
            segments.Add(rootSegment);
            
            _bioIkTarget = new GameObject("Bio IK Target").transform;
            _bioIkTarget.parent = transform;

            List<BioJoint> bioJoints = new();

            Transform parent = _bioIK.transform;

            int jointNumber = 1;

            for (int i = 0; i < RobotController.Joints.Length; i++)
            {
                if (!RobotController.Joints[i].HasMotion)
                {
                    continue;
                }
                
                GameObject go = new($"Joint {jointNumber++}")
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
                segment = segment.Create(_bioIK);
                segment.RenewRelations();
                segments.Add(segment);
                parent = go.transform;

                if (i == RobotController.Joints.Length - 1)
                {
                    Transform segmentTransform = segment.transform;
                    _bioIkTarget.position = segmentTransform.position;
                    _bioIkTarget.rotation = segmentTransform.rotation;
                    
                    _position = segment.AddObjective(ObjectiveType.Position) as Position;
                    if (_position != null)
                    {
                        _position.Create(segment);
                        _position.SetMaximumError(0);
                        _position.SetTargetTransform(_bioIkTarget);
                        _position.UpdateData();
                    }

                    _orientation = segment.AddObjective(ObjectiveType.Orientation) as Orientation;
                    if (_orientation != null)
                    {
                        _orientation.Create(segment);
                        _orientation.SetMaximumError(0);
                        _orientation.SetTargetTransform(_bioIkTarget);
                        _orientation.UpdateData();
                    }
                }

                BioJoint bioJoint = segment.AddJoint();
                bioJoint.Create(segment);
                bioJoint.JointType = RobotController.Joints[i].Type == ArticulationJointType.PrismaticJoint ? JointType.Translational : JointType.Rotational;
                bioJoint.SetOrientation(Vector3.zero);
                bioJoint.SetAnchor(Vector3.zero);
                bioJoints.Add(bioJoint);

                bioJoint.X.Constrained = true;
                if (RobotController.Joints[i].XMotion)
                {
                    bioJoint.X.SetEnabled(true);
                    if (bioJoint.JointType == JointType.Translational)
                    {
                        bioJoint.X.SetLowerLimit(RobotController.Joints[i].LimitX.Lower);
                        bioJoint.X.SetUpperLimit(RobotController.Joints[i].LimitX.Upper);
                    }
                    else
                    {
                        bioJoint.X.SetLowerLimit(RobotController.Joints[i].LimitX.Lower * Mathf.Rad2Deg);
                        bioJoint.X.SetUpperLimit(RobotController.Joints[i].LimitX.Upper * Mathf.Rad2Deg);
                    }

                    bioJoint.X.ProcessMotion(MotionType.Instantaneous);
                }
                else
                {
                    bioJoint.X.SetEnabled(false);
                }

                bioJoint.Y.Constrained = true;
                if (RobotController.Joints[i].YMotion)
                {
                    bioJoint.Y.SetEnabled(true);
                    if (bioJoint.JointType == JointType.Translational)
                    {
                        bioJoint.Y.SetLowerLimit(RobotController.Joints[i].LimitY.Lower);
                        bioJoint.Y.SetUpperLimit(RobotController.Joints[i].LimitY.Upper);
                    }
                    else
                    {
                        bioJoint.Y.SetLowerLimit(RobotController.Joints[i].LimitY.Lower * Mathf.Rad2Deg);
                        bioJoint.Y.SetUpperLimit(RobotController.Joints[i].LimitY.Upper * Mathf.Rad2Deg);
                    }
                    
                    bioJoint.Y.ProcessMotion(MotionType.Instantaneous);
                }
                else
                {
                    bioJoint.Y.SetEnabled(false);
                }
                
                bioJoint.Z.Constrained = true;
                if (RobotController.Joints[i].ZMotion)
                {
                    bioJoint.Z.SetEnabled(true);
                    if (bioJoint.JointType == JointType.Translational)
                    {
                        bioJoint.Z.SetLowerLimit(RobotController.Joints[i].LimitZ.Lower);
                        bioJoint.Z.SetUpperLimit(RobotController.Joints[i].LimitZ.Upper);
                    }
                    else
                    {
                        bioJoint.Z.SetLowerLimit(RobotController.Joints[i].LimitZ.Lower * Mathf.Rad2Deg);
                        bioJoint.Z.SetUpperLimit(RobotController.Joints[i].LimitZ.Upper * Mathf.Rad2Deg);
                    }
                    
                    bioJoint.Z.ProcessMotion(MotionType.Instantaneous);
                }
                else
                {
                    bioJoint.Z.SetEnabled(false);
                }
                
                bioJoint.PrecaptureAnimation();
                bioJoint.PostcaptureAnimation();
                bioJoint.UpdateData();
                bioJoint.ProcessMotion();
            }
            
            _bioIK.Refresh(false);

            _bioIK.PrecaptureAnimation(_bioIK.Root);
            _bioIK.PostcaptureAnimation(_bioIK.Root);

            for (int i = 0; i < segments.Count; i++)
            {
                _bioIK.SelectedSegment = segments[i];
            }

            _bioIK.Refresh();

            _bioJoints = bioJoints.ToArray();
        }
    }
}