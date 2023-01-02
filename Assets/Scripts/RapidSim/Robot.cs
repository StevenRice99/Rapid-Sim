using System;
using System.Collections.Generic;
using System.Linq;
using RapidSim.BioIK;
using RapidSim.Networks;
using Unity.Mathematics;
using UnityEngine;
using Random = UnityEngine.Random;

namespace RapidSim
{
    [DisallowMultipleComponent]
    public class Robot : MonoBehaviour
    {
        [Header("Robot Settings")]
        [Tooltip("How accurate in meters the robot can repeat a movement.")]
        [Min(0)]
        [SerializeField]
        private double repeatability = 8e-5;
        
        [Header("Bio IK Settings")]
        [Tooltip("The number of generations for a Bio IK evolution.")]
        [Min(1)]
        [SerializeField]
        private int generations = 5;
        
        [Min(1)]
        [Tooltip("The population size of each generation during Bio IK evolution.")]
        [SerializeField]
        private int populationSize = 120;
        
        [Min(1)]
        [Tooltip("The number of elites in each generation during Bio IK evolution.")]
        [SerializeField]
        private int elites = 3;
        
        [Min(1)]
        [Tooltip("The number of times to run the Bio IK algorithm when attempting to find an optimal move.")]
        [SerializeField]
        private int optimizeAttempts = 100;
        
        [Header("Neural Network Settings")]
        [Tooltip("The neural network to control the robot.")]
        [SerializeField]
        private NeuralNetwork network;
        
        [Tooltip("Click to train the neural network to control the robot.")]
        [SerializeField]
        private bool train;
        
        public BioIkJoint RootJoint { get; private set; }

        private ArticulationBody Root => _joints[0].Joint;

        private Transform LastJoint => _joints[^1].transform;

        private List<float> _home;

        private List<float> _zeros;

        private List<float> _targets;

        private bool _move;

        private float[] _maxSpeeds;

        private float[] _currentSpeeds;

        private JointLimit[] _limits;

        private float _chainLength;

        private RobotJoint[] _joints;

        private BioIkJoint[] _bioIkJoints;

        private BioIkJoint.Motion[] _motions;

        private Transform Objective => LastJoint.transform;

        private Transform _lastBioSegment;

        private void OnValidate()
        {
            if (elites > populationSize)
            {
                elites = populationSize;
            }
        }

        public void Start()
        {
            RobotJoint rootJoint = GetComponent<RobotJoint>();
        
            RobotJoint[] children = GetComponentsInChildren<RobotJoint>();

            if (rootJoint == null)
            {
                if (children.Length == 0)
                {
                    Debug.LogError($"No articulation bodies attached to {name}.");
                    Destroy(gameObject);
                    return;
                }

                _joints = children;
            }
            else if (children.Length > 0)
            {
                _joints = new RobotJoint[children.Length + 1];
                _joints[0] = rootJoint;
                for (int i = 1; i < _joints.Length - 1; i++)
                {
                    _joints[i] = children[i - 1];
                }
            }
            else
            {
                _joints = new RobotJoint[1];
                _joints[0] = rootJoint;
            }

            _joints = _joints.OrderBy(j => j.Joint.index).ToArray();

            _chainLength = 0;
            List<JointLimit> limits = new();
            for (int i = 0; i < _joints.Length; i++)
            {
                limits.AddRange(_joints[i].Limits());
                if (i > 0)
                {
                    _chainLength += Vector3.Distance(_joints[i - 1].transform.position, _joints[i].transform.position);
                }
            }

            _limits = limits.ToArray();
            _home = GetJoints();
            GetMaxSpeeds();

            if (_home.Count != _limits.Length)
            {
                Debug.LogError($"Ensure all joints on {name} have limits defined.");
                Destroy(gameObject);
                return;
            }
        
            _zeros = new();
            for (int i = 0; i < _home.Count; i++)
            {
                _zeros.Add(0);
            }
            
            _currentSpeeds = new float[_maxSpeeds.Length];
            for (int i = 0; i < _currentSpeeds.Length; i++)
            {
                _currentSpeeds[i] = _maxSpeeds[i];
            }

            if (_home.Count != _maxSpeeds.Length)
            {
                Debug.LogError($"{name} has {_home.Count} degrees of freedom but {_maxSpeeds.Length} speeds defined.");
                Destroy(gameObject);
                return;
            }

            SetupBioIk();

            NeuralNetwork.Validate(this, network, _limits.Length + 7, _limits.Length);
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
                angles[i] = math.abs(angles[i] - _targets[i]);
                if (angles[i] / _maxSpeeds[i] > time)
                {
                    time = angles[i] / _maxSpeeds[i];
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

        private List<float> GetJoints()
        {
            List<float> angles = new();
            Root.GetJointPositions(angles);
            return angles;
        }

        private void GetMaxSpeeds()
        {
            List<float> speeds = new();
            
            for (int i = 0; i < _joints.Length; i++)
            {
                if (!_joints[i].HasMotion)
                {
                    continue;
                }

                if (_joints[i].XMotion)
                {
                    speeds.Add(_joints[i].SpeedX);
                }

                if (_joints[i].YMotion)
                {
                    speeds.Add(_joints[i].SpeedY);
                }

                if (_joints[i].ZMotion)
                {
                    speeds.Add(_joints[i].SpeedZ);
                }
            }

            _maxSpeeds = speeds.ToArray();
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
                degrees[i] = math.radians(degrees[i]);
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
            
            double[] results = JointsScaled(network.Forward(PrepareInputs(NetScaled(starting), position, rotation)));

            List<float> joints = new();

            for (int i = 0; i < results.Length; i++)
            {
                joints.Add((float) math.clamp(results[i], _limits[i].lower, _limits[i].upper));
            }
        
            // TODO: Finalize movement with Hybrid IK.

            return joints;
        }

        private double[] PrepareInputs(double[] joints, Vector3 position, Quaternion rotation)
        {
            double[] inputs = new double[joints.Length + 7];
            for (int i = 0; i < joints.Length; i++)
            {
                inputs[i] = joints[i];
            }
            position = RelativePosition(position) / _chainLength;
            inputs[joints.Length] = position.x;
            inputs[joints.Length + 1] = position.y;
            inputs[joints.Length + 2] = position.z;
            rotation = RelativeRotation(rotation);
            inputs[joints.Length + 3] = rotation.x;
            inputs[joints.Length + 4] = rotation.y;
            inputs[joints.Length + 5] = rotation.z;
            inputs[joints.Length + 6] = rotation.w;
            return inputs;
        }

        private double[] NetScaled(double[] joints)
        {
            for (int i = 0; i < joints.Length; i++)
            {
                joints[i] = (joints[i] - _limits[i].lower) / (_limits[i].upper - _limits[i].lower) * 2 - 1;
            }

            return joints;
        }

        private double[] JointsScaled(double[] joints)
        {
            for (int i = 0; i < joints.Length; i++)
            {
                joints[i] = math.clamp((joints[i] + 1) / 2 * (_limits[i].upper - _limits[i].lower) + _limits[i].lower, _limits[i].lower, _limits[i].upper);
            }

            return joints;
        }

        private Vector3 RelativePosition(Vector3 position) => Root.transform.InverseTransformPoint(position);

        private Quaternion RelativeRotation(Quaternion rotation) => Quaternion.Inverse(Root.transform.rotation) * rotation;

        public double[] BioIkOptimize(Vector3 position, Quaternion orientation)
        {
            BioIkEvolution evolution = Initialise();
            double[] solution = new double[evolution.GetModel().GetDoF()];
            
            List<float> joints = GetJoints();

            double[] starting = new double[joints.Count];
            double[] maxSpeeds = new double[starting.Length];
            for (int i = 0; i < starting.Length; i++)
            {
                starting[i] = joints[i];
                maxSpeeds[i] = _maxSpeeds[i];
            }

            double[] best = new double[starting.Length];
            for (int i = 0; i < best.Length; i++)
            {
                best[i] = starting[i];
            }

            double bestAccuracy = Accuracy(LastJoint.position, position, Root.transform.rotation, LastJoint.rotation, orientation);
            double bestTime = 0;

            for (int attempt = 0; attempt < optimizeAttempts; attempt++)
            {
                for (int i = 0; i < starting.Length; i++)
                {
                    _motions[i].SetTargetValue(starting[i]);
                }
            
                UpdateData();
                for (int i = 0; i < solution.Length; i++)
                {
                    solution[i] = evolution.GetModel().motionPointers[i].motion.GetTargetValue();
                }
            
                solution = evolution.Optimise(generations, solution, position, orientation);

                for (int i = 0; i < solution.Length; i++)
                {
                    BioIkJoint.Motion motion = evolution.GetModel().motionPointers[i].motion;
                    motion.SetTargetValue(solution[i], true);
                }

                ProcessMotion();
                double[] ending = new double[_motions.Length];
                for (int i = 0; i < _motions.Length; i++)
                {
                    ending[i] = _motions[i].GetTargetValue();
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
        
        public double[] BioIkSolve(Vector3 position, Quaternion orientation)
        {
            BioIkEvolution evolution = Initialise();
            double[] solution = new double[evolution.GetModel().GetDoF()];
            
            List<float> joints = GetJoints();

            double[] starting = new double[joints.Count];
            double[] maxSpeeds = new double[starting.Length];
            for (int i = 0; i < starting.Length; i++)
            {
                starting[i] = joints[i];
                maxSpeeds[i] = _maxSpeeds[i];
            }

            for (int i = 0; i < starting.Length; i++)
            {
                _motions[i].SetTargetValue(starting[i]);
            }
            
            UpdateData();
            for (int i = 0; i < solution.Length; i++)
            {
                solution[i] = evolution.GetModel().motionPointers[i].motion.GetTargetValue();
            }
            
            solution = evolution.Optimise(generations, solution, position, orientation);

            for (int i = 0; i < solution.Length; i++)
            {
                BioIkJoint.Motion motion = evolution.GetModel().motionPointers[i].motion;
                motion.SetTargetValue(solution[i], true);
            }

            ProcessMotion();
            double[] ending = new double[_motions.Length];
            for (int i = 0; i < _motions.Length; i++)
            {
                ending[i] = _motions[i].GetTargetValue();
            }

            return ending;
        }
        
        private double Rescaling => math.PI_DBL * math.PI_DBL / (_chainLength * _chainLength);

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

        private float[] RandomOrientation()
        {
            float[] joints = new float[_limits.Length];
            for (int i = 0; i < joints.Length; i++)
            {
                joints[i] = Random.Range(_limits[i].lower, _limits[i].upper);
            }

            return joints;
        }

        private void SetRandomOrientation()
        {
            SnapRadians(RandomOrientation().ToList());
        }

        private void Update()
        {
            if (!train)
            {
                return;
            }

            if (network == null || network.step >= network.maxSteps)
            {
                train = false;
                return;
            }

            float[] floats = RandomOrientation();
            double[] angles = new double[floats.Length];
            for (int i = 0; i < floats.Length; i++)
            {
                angles[i] = floats[i];
            }
            
            angles = NetScaled(angles);
            double[] inputs = new double[angles.Length + 7];
            for (int i = 0; i < angles.Length; i++)
            {
                inputs[i] = angles[i];
            }

            SetRandomOrientation();
            Physics.autoSimulation = false;
            Physics.Simulate(Time.fixedDeltaTime);
            Physics.autoSimulation = true;

            Vector3 position = Objective.position;
            Quaternion rotation = Objective.rotation;
            Vector3 relativePosition = RelativePosition(position);
            Quaternion relativeRotation = RelativeRotation(rotation);

            inputs[angles.Length] = relativePosition.x;
            inputs[angles.Length + 1] = relativePosition.y;
            inputs[angles.Length + 2] = relativePosition.z;
            inputs[angles.Length + 3] = relativeRotation.x;
            inputs[angles.Length + 4] = relativeRotation.y;
            inputs[angles.Length + 5] = relativeRotation.z;
            inputs[angles.Length + 6] = relativeRotation.w;
    
            double[] expected = NetScaled(BioIkOptimize(position, rotation));
            
            // TODO - Train neural network.

            Debug.Log($"Training {network.step} of {network.maxSteps} - {(float)network.step / network.maxSteps * 100}%.");
        }

        private void SetupBioIk()
        {
            List<BioIkJoint> bioIkJoints = new();
            List<BioIkJoint.Motion> motions = new();
            BioIkJoint previousJoint = null;
            Transform parent = transform;

            int jointNumber = 1;

            for (int i = 0; i < _joints.Length; i++)
            {
                if (!_joints[i].HasMotion)
                {
                    continue;
                }
                
                GameObject go = new($"Bio IK Joint {jointNumber++}")
                {
                    transform =
                    {
                        parent = parent,
                        position = _joints[i].transform.position,
                        rotation = _joints[i].transform.rotation
                    }
                };

                BioIkJoint bioIkJoint = go.AddComponent<BioIkJoint>().Create();
                if (previousJoint == null)
                {
                    RootJoint = bioIkJoint;
                }
                else
                {
                    bioIkJoint.parent = previousJoint;
                    previousJoint.child = bioIkJoint;
                }
                previousJoint = bioIkJoint;
                _lastBioSegment = bioIkJoint.transform;
                parent = go.transform;
                
                bioIkJoint.rotational = _joints[i].Type != ArticulationJointType.PrismaticJoint;
                bioIkJoint.SetOrientation(Vector3.zero);
                bioIkJoints.Add(bioIkJoint);

                if (_joints[i].XMotion)
                {
                    motions.Add(bioIkJoint.y);
                    bioIkJoint.y.enabled = true;
                    if (!bioIkJoint.rotational)
                    {
                        bioIkJoint.y.SetLowerLimit(_joints[i].LimitX.lower);
                        bioIkJoint.y.SetUpperLimit(_joints[i].LimitX.upper);
                    }
                    else
                    {
                        bioIkJoint.y.SetLowerLimit(math.degrees(_joints[i].LimitX.lower));
                        bioIkJoint.y.SetUpperLimit(math.degrees(_joints[i].LimitX.upper));
                    }
                }
                else
                {
                    bioIkJoint.y.enabled = false;
                }

                if (_joints[i].YMotion)
                {
                    motions.Add(bioIkJoint.z);
                    bioIkJoint.z.enabled = true;
                    if (!bioIkJoint.rotational)
                    {
                        bioIkJoint.z.SetLowerLimit(_joints[i].LimitY.lower);
                        bioIkJoint.z.SetUpperLimit(_joints[i].LimitY.upper);
                    }
                    else
                    {
                        bioIkJoint.z.SetLowerLimit(math.degrees(_joints[i].LimitY.lower));
                        bioIkJoint.z.SetUpperLimit(math.degrees(_joints[i].LimitY.upper));
                    }
                }
                else
                {
                    bioIkJoint.z.enabled = false;
                }
                
                if (_joints[i].ZMotion)
                {
                    motions.Add(bioIkJoint.x);
                    bioIkJoint.x.enabled = true;
                    if (!bioIkJoint.rotational)
                    {
                        bioIkJoint.x.SetLowerLimit(_joints[i].LimitZ.lower);
                        bioIkJoint.x.SetUpperLimit(_joints[i].LimitZ.upper);
                    }
                    else
                    {
                        bioIkJoint.x.SetLowerLimit(math.degrees(_joints[i].LimitZ.lower));
                        bioIkJoint.x.SetUpperLimit(math.degrees(_joints[i].LimitZ.upper));
                    }
                }
                else
                {
                    bioIkJoint.x.enabled = false;
                }
            }

            _bioIkJoints = bioIkJoints.ToArray();
            _motions = motions.ToArray();
        }

        private BioIkEvolution Initialise()
        {
            return new(new(this, Rescaling), populationSize, elites, Rescaling);
        }

        private void UpdateData()
        {
            for (int i = 0; i < _bioIkJoints.Length; i++)
            {
                _bioIkJoints[i].UpdateData();
            }
        }

        private void ProcessMotion()
        {
            for (int i = 0; i < _bioIkJoints.Length; i++)
            {
                _bioIkJoints[i].ProcessMotion();
            }
        }
    }
}