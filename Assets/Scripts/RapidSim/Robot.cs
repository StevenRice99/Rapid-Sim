using System;
using System.Collections.Generic;
using System.Linq;
using BioIK.Helpers;
using BioIK.Setup;
using RapidSim.Networks;
using Unity.Mathematics;
using UnityEngine;
using Random = UnityEngine.Random;

namespace RapidSim
{
    [DisallowMultipleComponent]
    public class Robot : MonoBehaviour
    {
        [Min(0)]
        [SerializeField]
        private double repeatability = 8e-5;
        
        [Min(1)]
        [SerializeField]
        private int bioIkAttempts = 100;
        
        [Min(1)]
        [SerializeField]
        private int bioIkGenerations = 5;
        
        [Min(1)]
        [SerializeField]
        private int bioIkPopulationSize = 120;
        
        [Min(1)]
        [SerializeField]
        private int bioIkElites = 3;
        
        [Min(1)]
        [SerializeField]
        private int maxSteps = 1;
        
        [SerializeField]
        private bool train;
        
        public ArticulationBody Root => Joints[0].Joint;

        public Transform LastJoint => Joints[^1].transform;

        private List<float> _home;

        private List<float> _zeros;

        private List<float> _targets;

        private bool _move;
        
        public float[] MaxSpeeds { get; private set; }

        private float[] _currentSpeeds;

        public JointLimit[] Limits { get; private set; }

        public float ChainLength { get; private set; }

        public RobotJoint[] Joints { get; private set; }
        
        [SerializeField]
        private NeuralNetworkData data;

        public NeuralNetwork Net { get; private set; }

        public int NetworkSteps => Net.step;

        private BioIK.BioIK _bioIK;

        private BioObjective _position;

        private bool _train;

        private BioJoint.Motion[] _motions;

        private Transform _lastBioSegment;

        public Transform Objective => LastJoint.transform;

        public void Start()
        {
            RobotJoint root = GetComponent<RobotJoint>();
        
            RobotJoint[] children = GetComponentsInChildren<RobotJoint>();

            if (root == null)
            {
                if (children.Length == 0)
                {
                    Debug.LogError($"No articulation bodies attached to {name}.");
                    Destroy(this);
                    return;
                }

                Joints = children;
            }
            else if (children.Length > 0)
            {
                Joints = new RobotJoint[children.Length + 1];
                ChainLength = Vector3.Distance(Root.transform.position, children[0].transform.position);
                Joints[0] = root;
                for (int i = 1; i < Joints.Length - 1; i++)
                {
                    Joints[i] = children[i - 1];
                }
            }
            else
            {
                Joints = new RobotJoint[1];
                Joints[0] = root;
            }

            Joints = Joints.OrderBy(j => j.Joint.index).ToArray();

            List<JointLimit> limits = new();
            for (int i = 0; i < Joints.Length; i++)
            {
                limits.AddRange(Joints[i].Limits());
            }

            Limits = limits.ToArray();
            _home = GetJoints();
            GetMaxSpeeds();

            if (_home.Count != Limits.Length)
            {
                Debug.LogError($"Ensure all joints on {name} have limits defined.");
            }
        
            _zeros = new();
            for (int i = 0; i < _home.Count; i++)
            {
                _zeros.Add(0);
            }
            
            _currentSpeeds = new float[MaxSpeeds.Length];
            for (int i = 0; i < _currentSpeeds.Length; i++)
            {
                _currentSpeeds[i] = MaxSpeeds[i];
            }

            if (_home.Count != MaxSpeeds.Length)
            {
                Debug.LogError($"{name} has {_home.Count} degrees of freedom but {MaxSpeeds.Length} speeds defined.");
            }
            
            if (data != null && data.HasModel)
            {
                Net = data.Load();
                return;
            }

            int s = GetJoints().Count;
            int[] layers = new int[s + 2];
            layers[^1] = s;
            s += 7;
            layers[0] = s;
            s *= 2;
            for (int i = 1; i < layers.Length - 1; i++)
            {
                layers[i] = s;
            }

            Net = new(layers);
            
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
        
        public void Move(GameObject target)
        {
            Move(target.transform);
        }

        public void Move(Component target)
        {
            Move(target.transform);
        }

        public void Move(Transform target)
        {
            Move(target.position, target.rotation);
        }

        public void Move(Vector3 position)
        {
            Move(position, LastJoint.rotation);
        }

        public void Move(Quaternion rotation)
        {
            Move(LastJoint.position, rotation);
        }

        public void Move(Vector3 position, Quaternion rotation)
        {
            MoveRadians(Solve(position, rotation));
        }

        public void Move(List<float> degrees)
        {
            MoveRadians(DegreesToRadians(degrees));
        }

        public void MoveRadians(List<float> radians)
        {
            _targets = radians;

            List<float> angles = GetJoints();
            float time = 0;
            for (int i = 0; i < angles.Count; i++)
            {
                angles[i] = Mathf.Abs(angles[i] - _targets[i]);
                if (angles[i] / MaxSpeeds[i] > time)
                {
                    time = angles[i] / MaxSpeeds[i];
                }
            }

            if (time > 0)
            {
                for (int i = 0; i < _currentSpeeds.Length; i++)
                {
                    _currentSpeeds[i] = angles[i] / time;
                }
            }

            _move = true;
        }
        
        public void Snap(GameObject target)
        {
            Snap(target.transform);
        }

        public void Snap(Component target)
        {
            Snap(target.transform);
        }

        public void Snap(Transform target)
        {
            Snap(target.position, target.rotation);
        }

        public void Snap(Vector3 position)
        {
            Snap(position, LastJoint.rotation);
        }

        public void Snap(Quaternion rotation)
        {
            Snap(LastJoint.position, rotation);
        }
    
        public void Snap(Vector3 position, Quaternion rotation)
        {
            SnapRadians(Solve(position, rotation));
        }

        public void Snap(List<float> degrees)
        {
            SnapRadians(DegreesToRadians(degrees));
        }

        public void SnapRadians(List<float> radians)
        {
            _move = false;
            Stop(radians);
        }

        public void MoveHome()
        {
            MoveRadians(_home);
        }

        public void SnapHome()
        {
            SnapRadians(_home);
        }

        public List<float> GetJoints()
        {
            List<float> angles = new();
            Root.GetJointPositions(angles);
            return angles;
        }

        private void GetMaxSpeeds()
        {
            List<float> speeds = new();
            
            for (int i = 0; i < Joints.Length; i++)
            {
                if (!Joints[i].HasMotion)
                {
                    continue;
                }

                if (Joints[i].XMotion)
                {
                    speeds.Add(Joints[i].SpeedX);
                }

                if (Joints[i].YMotion)
                {
                    speeds.Add(Joints[i].SpeedY);
                }

                if (Joints[i].ZMotion)
                {
                    speeds.Add(Joints[i].SpeedZ);
                }
            }

            MaxSpeeds = speeds.ToArray();
        }

        private void FixedUpdate()
        {
            if (!_move)
            {
                return;
            }

            _move = false;

            List<float> delta = GetJoints();
            for (int i = 0; i < delta.Count; i++)
            {
                if (delta[i] >= _targets[i])
                {
                    delta[i] -= _currentSpeeds[i] * Time.fixedDeltaTime;
                    if (delta[i] <= _targets[i])
                    {
                        delta[i] = _targets[i];
                    }
                    else
                    {
                        _move = true;
                    }
                }
                else
                {
                    delta[i] += _currentSpeeds[i] * Time.fixedDeltaTime;
                    if (delta[i] >= _targets[i])
                    {
                        delta[i] = _targets[i];
                    }
                    else
                    {
                        _move = true;
                    }
                }
            }

            if (_move)
            {
                Root.SetDriveTargets(delta);
            }
            else
            {
                Stop(delta);
            }
        }

        private void Stop(IEnumerable<float> radians)
        {
            List<float> list = radians.ToList();
            Root.SetDriveTargets(list);
            Root.SetJointVelocities(_zeros);
            Root.SetJointAccelerations(_zeros);
            Root.SetJointForces(_zeros);
            Root.SetJointPositions(list);
        }

        private static List<float> DegreesToRadians(List<float> degrees)
        {
            for (int i = 0; i < degrees.Count; i++)
            {
                degrees[i] *= Mathf.Deg2Rad;
            }

            return degrees;
        }
        
                private List<float> Solve(Vector3 position, Quaternion rotation)
        {
            List<float> original = GetJoints();
            double[] starting = new double[original.Count];
            for (int i = 0; i < starting.Length; i++)
            {
                starting[i] = original[i];
            }
            
            double[] results = JointsScaled(Net.Forward(PrepareInputs(NetScaled(starting), position, rotation)));

            List<float> joints = new();

            for (int i = 0; i < results.Length; i++)
            {
                joints.Add((float) math.clamp(results[i], Limits[i].Lower, Limits[i].Upper));;
            }
        
            // TODO: Finalize movement with Hybrid IK.

            return joints;
        }

        public double[] PrepareInputs(double[] joints, Vector3 position, Quaternion rotation)
        {
            double[] inputs = new double[7 + joints.Length];
            position = RelativePosition(position) / ChainLength;
            inputs[0] = position.x;
            inputs[1] = position.y;
            inputs[2] = position.z;
            rotation = RelativeRotation(rotation);
            inputs[3] = rotation.x;
            inputs[4] = rotation.y;
            inputs[5] = rotation.z;
            inputs[6] = rotation.w;
            for (int i = 0; i < joints.Length; i++)
            {
                inputs[i + 7] = joints[i];
            }
            return inputs;
        }
    
        public double[] NetScaled(double[] joints)
        {
            for (int i = 0; i < joints.Length; i++)
            {
                joints[i] = (joints[i] - Limits[i].Lower) / (Limits[i].Upper - Limits[i].Lower);
            }

            return joints;
        }
    
        public double[] JointsScaled(double[] joints)
        {
            for (int i = 0; i < joints.Length; i++)
            {
                joints[i] = math.clamp(joints[i] * (Limits[i].Upper - Limits[i].Lower) + Limits[i].Lower, Limits[i].Lower, Limits[i].Upper);
            }

            return joints;
        }

        public Vector3 RelativePosition(Vector3 position) => Root.transform.InverseTransformPoint(position);
    
        public Quaternion RelativeRotation(Quaternion rotation) => Quaternion.Inverse(Root.transform.rotation) * rotation;
        
                public double[] BioIkSolve(Vector3 position, Quaternion orientation)
        {
            _bioIK.DeInitialise();
            _bioIK.Initialise();
            _bioIK.solution = new double[_bioIK.Evolution.GetModel().GetDoF()];
            
            _position.SetTargetPosition(position);
            _position.SetTargetRotation(orientation);
            
            List<float> joints = GetJoints();

            double[] starting = new double[joints.Count];
            double[] maxSpeeds = new double[starting.Length];
            for (int i = 0; i < starting.Length; i++)
            {
                starting[i] = joints[i];
                maxSpeeds[i] = MaxSpeeds[i];
            }

            double[] best = new double[starting.Length];
            for (int i = 0; i < best.Length; i++)
            {
                best[i] = starting[i];
            }
            
            double bestAccuracy = Accuracy(LastJoint.position, position, Root.transform.rotation, LastJoint.rotation, orientation);
            double bestTime = 0;

            for (int j = 0; j < bioIkAttempts; j++)
            {
                for (int i = 0; i < starting.Length; i++)
                {
                    _motions[i].SetTargetValue(starting[i]);
                }
            
                BioIK.BioIK.UpdateData(_bioIK.root);
            
                for (int i = 0; i < _bioIK.solution.Length; i++)
                {
                    _bioIK.solution[i] = _bioIK.Evolution.GetModel().MotionPointers[i].Motion.GetTargetValue(true);
                }
            
                _bioIK.solution = _bioIK.Evolution.Optimise(_bioIK.generations, _bioIK.solution);

                for (int i = 0; i< _bioIK.solution.Length; i++)
                {
                    BioJoint.Motion motion = _bioIK.Evolution.GetModel().MotionPointers[i].Motion;
                    motion.SetTargetValue(_bioIK.solution[i], true);
                }

                BioIK.BioIK.ProcessMotion(_bioIK.root);

                double[] ending = new double[_motions.Length];
                for (int i = 0; i < _motions.Length; i++)
                {
                    ending[i] = _motions[i].GetTargetValue(true);
                }

                double accuracy = Accuracy(_lastBioSegment.position, position, Root.transform.rotation, _lastBioSegment.rotation, orientation);
                double time = CalculateTime(starting, ending, maxSpeeds);

                if (bestAccuracy > repeatability && accuracy < bestAccuracy)
                {
                    best = ending;
                    bestAccuracy = accuracy;
                    bestTime = time;
                    continue;
                }

                if (accuracy <= repeatability && time < bestTime)
                {
                    best = ending;
                    bestAccuracy = accuracy;
                    bestTime = time;
                }
            }

            return best;
        }

        private static double Accuracy(Vector3 currentPosition, Vector3 goalPosition, Quaternion rootRotation, Quaternion currentEndRotation, Quaternion goalEndRotation)
        {
            return Vector3.Distance(currentPosition, goalPosition) + Quaternion.Angle(goalEndRotation, Quaternion.Inverse(rootRotation) * currentEndRotation);
        }

        private static double CalculateTime(double[] starting, double[] ending, double[] maxSpeeds)
        {
            double longestTime = 0;
            for (int i = 0; i < starting.Length; i++)
            {
                double time = Math.Abs(starting[i] - ending[i]) / maxSpeeds[i];
                if (time> longestTime)
                {
                    longestTime = time;
                }
            }

            return longestTime;
        }

        public void Train(Vector3 position, Quaternion rotation, double[] joints, double[] expected)
        {
            Net.Train(PrepareInputs(NetScaled(joints), position, rotation), NetScaled(expected));
        }

        public float[] RandomOrientation()
        {
            float[] joints = new float[Limits.Length];
            for (int i = 0; i < joints.Length; i++)
            {
                joints[i] = Random.Range(Limits[i].Lower, Limits[i].Upper);
            }

            return joints;
        }

        public void SetRandomOrientation()
        {
            SnapRadians(RandomOrientation().ToList());
        }

        private void Update()
        {
            if (NetworkSteps >= maxSteps)
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
            Transform rootTransform = Root.transform;
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
            _bioIK.generations = bioIkGenerations;
            _bioIK.SetPopulationSize(bioIkPopulationSize);
            _bioIK.SetElites(bioIkElites);

            _bioIK.Refresh(false);
            BioSegment rootSegment = bioIkHolder.GetComponent<BioSegment>();
            rootSegment = rootSegment.Create(_bioIK);
            rootSegment.RenewRelations();
            _lastBioSegment = rootSegment.transform;

            List<BioJoint.Motion> motions = new();

            Transform parent = _bioIK.transform;

            int jointNumber = 1;

            for (int i = 0; i < Joints.Length; i++)
            {
                if (!Joints[i].HasMotion)
                {
                    continue;
                }
                
                GameObject go = new($"Joint {jointNumber++}")
                {
                    transform =
                    {
                        parent = parent,
                        position = Joints[i].transform.position,
                        rotation = Joints[i].transform.rotation
                    }
                };

                _bioIK.Refresh(false);
                BioSegment segment = go.GetComponent<BioSegment>();
                segment = segment.Create(_bioIK);
                segment.RenewRelations();
                _lastBioSegment = segment.transform;
                parent = go.transform;

                if (i == Joints.Length - 1)
                {
                    Transform segmentTransform = segment.transform;
                    
                    _position = segment.AddObjective();
                    if (_position != null)
                    {
                        _position.Create(segment);
                        _position.SetTargetPosition(segmentTransform.position);
                    }
                }

                BioJoint bioJoint = segment.AddJoint();
                bioJoint.Create(segment);
                bioJoint.jointType = Joints[i].Type == ArticulationJointType.PrismaticJoint ? JointType.Translational : JointType.Rotational;
                bioJoint.SetOrientation(Vector3.zero);

                if (Joints[i].XMotion)
                {
                    motions.Add(bioJoint.y);
                    bioJoint.y.SetEnabled(true);
                    if (bioJoint.jointType == JointType.Translational)
                    {
                        bioJoint.y.SetLowerLimit(Joints[i].LimitX.Lower);
                        bioJoint.y.SetUpperLimit(Joints[i].LimitX.Upper);
                    }
                    else
                    {
                        bioJoint.y.SetLowerLimit(Joints[i].LimitX.Lower * Mathf.Rad2Deg);
                        bioJoint.y.SetUpperLimit(Joints[i].LimitX.Upper * Mathf.Rad2Deg);
                    }
                }
                else
                {
                    bioJoint.y.SetEnabled(false);
                }

                if (Joints[i].YMotion)
                {
                    motions.Add(bioJoint.z);
                    bioJoint.z.SetEnabled(true);
                    if (bioJoint.jointType == JointType.Translational)
                    {
                        bioJoint.z.SetLowerLimit(Joints[i].LimitY.Lower);
                        bioJoint.z.SetUpperLimit(Joints[i].LimitY.Upper);
                    }
                    else
                    {
                        bioJoint.z.SetLowerLimit(Joints[i].LimitY.Lower * Mathf.Rad2Deg);
                        bioJoint.z.SetUpperLimit(Joints[i].LimitY.Upper * Mathf.Rad2Deg);
                    }
                }
                else
                {
                    bioJoint.z.SetEnabled(false);
                }
                
                if (Joints[i].ZMotion)
                {
                    motions.Add(bioJoint.x);
                    bioJoint.x.SetEnabled(true);
                    if (bioJoint.jointType == JointType.Translational)
                    {
                        bioJoint.x.SetLowerLimit(Joints[i].LimitZ.Lower);
                        bioJoint.x.SetUpperLimit(Joints[i].LimitZ.Upper);
                    }
                    else
                    {
                        bioJoint.x.SetLowerLimit(Joints[i].LimitZ.Lower * Mathf.Rad2Deg);
                        bioJoint.x.SetUpperLimit(Joints[i].LimitZ.Upper * Mathf.Rad2Deg);
                    }
                }
                else
                {
                    bioJoint.x.SetEnabled(false);
                }
            }

            _bioIK.Refresh();

            _motions = motions.ToArray();
        }
    }
}