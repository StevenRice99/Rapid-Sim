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
        private int bioIkAttempts = 20;
        
        [Min(1)]
        [SerializeField]
        private int maxSteps = 1;
        
        [SerializeField]
        private bool train;
        
        public RobotController RobotController { get; private set; }

        public RobotSolver RobotSolver { get; private set; }

        private BioIK.BioIK _bioIK;

        private Position _position;

        private Orientation _orientation;

        private bool _train;

        private BioJoint.Motion[] _motions;

        private JointValue[] _jointValues;

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
            _position.SetTargetPosition(position);
            _orientation.SetTargetRotation(orientation);
            
            float[] starting = RobotController.GetJoints().ToArray();

            float[] best = null;
            float bestDistance = 0;
            float bestSpeed = 0;

            for (int j = 0; j < bioIkAttempts; j++)
            {
                for (int i = 0; i < starting.Length; i++)
                {
                    _motions[i].SetTargetValue(starting[i]);
                    _jointValues[i].SetTargetValue(starting[i]);
                    _jointValues[i].SetWeight(0);
                }
            
                _bioIK.UpdateData(_bioIK.Root);
            
                for(int i = 0; i < _bioIK.Solution.Length; i++) {
                    _bioIK.Solution[i] = _bioIK.Evolution.GetModel().MotionPtrs[i].Motion.GetTargetValue(true);
                }
            
                _bioIK.Solution = _bioIK.Evolution.Optimise(_bioIK.GetGenerations(), _bioIK.Solution);

                for (int i = 0; i< _bioIK.Solution.Length; i++)
                {
                    BioJoint.Motion motion = _bioIK.Evolution.GetModel().MotionPtrs[i].Motion;
                    motion.SetTargetValue(_bioIK.Solution[i], true);
                }

                _bioIK.ProcessMotion(_bioIK.Root);

                float[] ending = new float[_motions.Length];
                for (int i = 0; i < _motions.Length; i++)
                {
                    ending[i] = (float) _motions[i].GetTargetValue(true);
                }

                float distance = Vector3.Distance(_motions[^1].Joint.transform.position, position);
                float speed = CalculateSpeed(starting, ending, RobotController.MaxSpeeds);
                if (best == null)
                {
                    best = ending;
                    bestDistance = distance;
                    bestSpeed = speed;
                    continue;
                }

                if (distance < bestDistance)
                {
                    best = ending;
                    bestDistance = distance;
                    bestSpeed = speed;
                    continue;
                }

                if (distance <= bestDistance && speed < bestSpeed)
                {
                    best = ending;
                    bestDistance = distance;
                    bestSpeed = speed;
                }
            }

            return best;
        }

        private float CalculateSpeed(float[] starting, float[] ending, float[] maxSpeeds)
        {
            float time = 0;
            for (int i = 0; i < starting.Length; i++)
            {
                starting[i] = Mathf.Abs(starting[i] - ending[i]);
                if (starting[i] / maxSpeeds[i] > time)
                {
                    time = starting[i] / maxSpeeds[i];
                }
            }

            return time;
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
            _bioIK.SetGenerations(5);
            _bioIK.SetPopulationSize(120);
            _bioIK.SetElites(3);

            _bioIK.Refresh(false);
            BioSegment rootSegment = bioIkHolder.GetComponent<BioSegment>();
            rootSegment = rootSegment.Create(_bioIK);
            rootSegment.RenewRelations();

            List<BioJoint.Motion> motions = new();
            List<JointValue> jointValues = new();

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
                parent = go.transform;

                if (i == RobotController.Joints.Length - 1)
                {
                    Transform segmentTransform = segment.transform;
                    
                    _position = segment.AddObjective(ObjectiveType.Position) as Position;
                    if (_position != null)
                    {
                        _position.Create(segment);
                        _position.SetWeight(10);
                        _position.SetMaximumError(0);
                        _position.SetTargetPosition(segmentTransform.position);
                    }

                    _orientation = segment.AddObjective(ObjectiveType.Orientation) as Orientation;
                    if (_orientation != null)
                    {
                        _orientation.Create(segment);
                        _orientation.SetWeight(10);
                        _orientation.SetMaximumError(0);
                        _orientation.SetTargetRotation(segmentTransform.rotation);
                    }
                }

                BioJoint bioJoint = segment.AddJoint();
                bioJoint.Create(segment);
                bioJoint.JointType = RobotController.Joints[i].Type == ArticulationJointType.PrismaticJoint ? JointType.Translational : JointType.Rotational;
                bioJoint.SetOrientation(Vector3.zero);
                bioJoint.SetAnchor(Vector3.zero);

                bioJoint.Y.Constrained = true;
                if (RobotController.Joints[i].XMotion)
                {
                    motions.Add(bioJoint.Y);
                    bioJoint.Y.SetEnabled(true);
                    if (bioJoint.JointType == JointType.Translational)
                    {
                        bioJoint.Y.SetLowerLimit(RobotController.Joints[i].LimitX.Lower);
                        bioJoint.Y.SetUpperLimit(RobotController.Joints[i].LimitX.Upper);
                    }
                    else
                    {
                        bioJoint.Y.SetLowerLimit(RobotController.Joints[i].LimitX.Lower * Mathf.Rad2Deg);
                        bioJoint.Y.SetUpperLimit(RobotController.Joints[i].LimitX.Upper * Mathf.Rad2Deg);
                    }

                    JointValue jointValue = segment.AddObjective(ObjectiveType.JointValue) as JointValue;
                    if (jointValue != null)
                    {
                        jointValue.Create(segment);
                        jointValue.SetWeight(0);
                        jointValue.X = false;
                        jointValue.Y = true;
                        jointValue.Z = false;
                        jointValue.TargetValue = 0;
                        jointValues.Add(jointValue);
                    }
                }
                else
                {
                    bioJoint.Y.SetEnabled(false);
                }

                bioJoint.Z.Constrained = true;
                if (RobotController.Joints[i].YMotion)
                {
                    motions.Add(bioJoint.Z);
                    bioJoint.Z.SetEnabled(true);
                    if (bioJoint.JointType == JointType.Translational)
                    {
                        bioJoint.Z.SetLowerLimit(RobotController.Joints[i].LimitY.Lower);
                        bioJoint.Z.SetUpperLimit(RobotController.Joints[i].LimitY.Upper);
                    }
                    else
                    {
                        bioJoint.Z.SetLowerLimit(RobotController.Joints[i].LimitY.Lower * Mathf.Rad2Deg);
                        bioJoint.Z.SetUpperLimit(RobotController.Joints[i].LimitY.Upper * Mathf.Rad2Deg);
                    }
                    
                    JointValue jointValue = segment.AddObjective(ObjectiveType.JointValue) as JointValue;
                    if (jointValue != null)
                    {
                        jointValue.Create(segment);
                        jointValue.SetWeight(0);
                        jointValue.X = false;
                        jointValue.Y = false;
                        jointValue.Z = true;
                        jointValue.TargetValue = 0;
                        jointValues.Add(jointValue);
                    }
                }
                else
                {
                    bioJoint.Z.SetEnabled(false);
                }
                
                bioJoint.X.Constrained = true;
                if (RobotController.Joints[i].ZMotion)
                {
                    motions.Add(bioJoint.X);
                    bioJoint.X.SetEnabled(true);
                    if (bioJoint.JointType == JointType.Translational)
                    {
                        bioJoint.X.SetLowerLimit(RobotController.Joints[i].LimitZ.Lower);
                        bioJoint.X.SetUpperLimit(RobotController.Joints[i].LimitZ.Upper);
                    }
                    else
                    {
                        bioJoint.X.SetLowerLimit(RobotController.Joints[i].LimitZ.Lower * Mathf.Rad2Deg);
                        bioJoint.X.SetUpperLimit(RobotController.Joints[i].LimitZ.Upper * Mathf.Rad2Deg);
                    }
                    
                    JointValue jointValue = segment.AddObjective(ObjectiveType.JointValue) as JointValue;
                    if (jointValue != null)
                    {
                        jointValue.Create(segment);
                        jointValue.SetWeight(0);
                        jointValue.X = true;
                        jointValue.Y = false;
                        jointValue.Z = false;
                        jointValue.TargetValue = 0;
                        jointValues.Add(jointValue);
                    }
                }
                else
                {
                    bioJoint.X.SetEnabled(false);
                }
            }

            _bioIK.Refresh();

            _motions = motions.ToArray();
            _jointValues = jointValues.ToArray();
        }
    }
}