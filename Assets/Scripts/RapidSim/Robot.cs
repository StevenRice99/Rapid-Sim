using System;
using System.Collections.Generic;
using System.Linq;
using RapidSim.BioIK;
using Unity.Mathematics;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;
using Unity.MLAgents.Sensors;
using UnityEngine;
using Random = UnityEngine.Random;

namespace RapidSim
{
    [RequireComponent(typeof(BehaviorParameters))]
    [DisallowMultipleComponent]
    public class Robot : Agent
    {
        [Header("Robot Settings")]
        [Tooltip("How accurate in meters the robot can repeat a movement.")]
        [Min(0)]
        [SerializeField]
        private float repeatability = 8e-5f;
        
        [Header("Bio IK Settings")]
        [Tooltip("The number of generations for a Bio IK evolution.")]
        [Min(1)]
        [SerializeField]
        private int generations = 5;
        
        [Tooltip("The population size of each generation during Bio IK evolution.")]
        [Min(1)]
        [SerializeField]
        private int populationSize = 120;
        
        [Tooltip("The number of elites in each generation during Bio IK evolution.")]
        [Min(1)]
        [SerializeField]
        private int elites = 3;
        
        [Tooltip("The number of times to run the Bio IK algorithm when attempting to find an optimal move.")]
        [Min(1)]
        [SerializeField]
        private int optimizeAttempts = 10;

        public int PopulationSize => populationSize;

        public int Elites => elites;

        private ArticulationBody Root => _joints[0].Joint;

        private Transform LastJoint => _joints[^1].transform;

        private List<float> _home;

        private List<float> _middle;

        private List<float> _zeros;

        private List<float> _targets;

        private bool _move;

        private float[] _maxSpeeds;

        private float[] _currentSpeeds;

        private JointLimit[] _limits;

        private RobotJoint[] _joints;

        public BioIkJoint[] BioIkJoints { get; private set; }
        
        public float Rescaling { get; private set; }

        private BioIkJoint.Motion[] _motions;

        private Transform _lastBioIkJoint;

        private float _chainLength;

        private Vector3 _mlAgentsPos;

        private Quaternion _mlAgentsRot;

        private float _maxTime;

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

            _middle = new();
            _chainLength = 0;
            List<JointLimit> limits = new();
            for (int i = 0; i < _joints.Length; i++)
            {
                List<JointLimit> limit = _joints[i].Limits();
                for (int j = 0; j < limit.Count; j++)
                {
                    _middle.Add((limit[j].lower + limit[j].upper) / 2);
                }
                limits.AddRange(limit);
                if (i > 0)
                {
                    _chainLength += Vector3.Distance(_joints[i - 1].transform.position, _joints[i].transform.position);
                }
            }

            Rescaling = math.PI * math.PI / (_chainLength * _chainLength);

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

            _maxTime = float.MinValue;
            for (int i = 0; i < _maxSpeeds.Length; i++)
            {
                float time = (_limits[i].upper - _limits[i].lower) / _maxSpeeds[i];
                if (time > _maxTime)
                {
                    _maxTime = time;
                }
            }

            if (_home.Count != _maxSpeeds.Length)
            {
                Debug.LogError($"{name} has {_home.Count} degrees of freedom but {_maxSpeeds.Length} speeds defined.");
                Destroy(gameObject);
                return;
            }

            List<BioIkJoint> bioIkJoints = new();
            List<BioIkJoint.Motion> motions = new();
            BioIkJoint previousJoint = null;
            Transform parent = transform;

            int jointNumber = 1;

            foreach (RobotJoint j in _joints)
            {
                if (!j.HasMotion)
                {
                    continue;
                }

                Transform jointTransform = j.transform;
                GameObject go = new($"Bio IK Joint {jointNumber++}")
                {
                    transform =
                    {
                        parent = parent,
                        position = jointTransform.position,
                        rotation = jointTransform.rotation
                    }
                };

                BioIkJoint bioIkJoint = go.AddComponent<BioIkJoint>();
                if (previousJoint != null)
                {
                    bioIkJoint.parent = previousJoint;
                    previousJoint.child = bioIkJoint;
                }
                bioIkJoint.Setup();
                previousJoint = bioIkJoint;
                _lastBioIkJoint = bioIkJoint.transform;
                parent = go.transform;
                
                bioIkJoint.rotational = j.Type != ArticulationJointType.PrismaticJoint;
                bioIkJoint.SetOrientation(Vector3.zero);
                bioIkJoints.Add(bioIkJoint);

                if (j.XMotion)
                {
                    motions.Add(bioIkJoint.y);
                    bioIkJoint.y.enabled = true;
                    if (!bioIkJoint.rotational)
                    {
                        bioIkJoint.y.SetLowerLimit(j.LimitX.lower);
                        bioIkJoint.y.SetUpperLimit(j.LimitX.upper);
                    }
                    else
                    {
                        bioIkJoint.y.SetLowerLimit(math.degrees(j.LimitX.lower));
                        bioIkJoint.y.SetUpperLimit(math.degrees(j.LimitX.upper));
                    }
                }
                else
                {
                    bioIkJoint.y.enabled = false;
                }

                if (j.YMotion)
                {
                    motions.Add(bioIkJoint.z);
                    bioIkJoint.z.enabled = true;
                    if (!bioIkJoint.rotational)
                    {
                        bioIkJoint.z.SetLowerLimit(j.LimitY.lower);
                        bioIkJoint.z.SetUpperLimit(j.LimitY.upper);
                    }
                    else
                    {
                        bioIkJoint.z.SetLowerLimit(math.degrees(j.LimitY.lower));
                        bioIkJoint.z.SetUpperLimit(math.degrees(j.LimitY.upper));
                    }
                }
                else
                {
                    bioIkJoint.z.enabled = false;
                }
                
                if (j.ZMotion)
                {
                    motions.Add(bioIkJoint.x);
                    bioIkJoint.x.enabled = true;
                    if (!bioIkJoint.rotational)
                    {
                        bioIkJoint.x.SetLowerLimit(j.LimitZ.lower);
                        bioIkJoint.x.SetUpperLimit(j.LimitZ.upper);
                    }
                    else
                    {
                        bioIkJoint.x.SetLowerLimit(math.degrees(j.LimitZ.lower));
                        bioIkJoint.x.SetUpperLimit(math.degrees(j.LimitZ.upper));
                    }
                }
                else
                {
                    bioIkJoint.x.enabled = false;
                }
            }

            BioIkJoints = bioIkJoints.ToArray();
            _motions = motions.ToArray();

            UpdateData();
            
            BehaviorParameters parameters = GetComponent<BehaviorParameters>();
            if (parameters == null)
            {
                parameters = gameObject.AddComponent<BehaviorParameters>();
            }

            MaxStep = Academy.Instance.IsCommunicatorOn ? 1 : 0;

            parameters.BrainParameters.VectorObservationSize = 7 + _limits.Length;
            parameters.BrainParameters.NumStackedVectorObservations = 1;
            ActionSpec spec = parameters.BrainParameters.ActionSpec;
            spec.NumContinuousActions = _home.Count;
            spec.BranchSizes = Array.Empty<int>();
            parameters.BrainParameters.ActionSpec = spec;
        }

        public override void OnEpisodeBegin()
        {
            if (!Academy.Instance.IsCommunicatorOn)
            {
                return;
            }
            
            _mlAgentsPos = Random.insideUnitSphere * _chainLength + transform.position;
            _mlAgentsRot = Random.rotation;
            
            SetRandomJoints();
            PhysicsStep();
            
            RequestDecision();
        }

        public override void CollectObservations(VectorSensor sensor)
        {
            sensor.AddObservation(RelativePosition(_mlAgentsPos));
            sensor.AddObservation(RelativeRotation(_mlAgentsRot));
            sensor.AddObservation(NetScaledJoints());
        }

        public override void OnActionReceived(ActionBuffers actions)
        {
            List<float> joints = actions.ContinuousActions.ToList();

            joints = JointsScaled(joints);

            if (_move)
            {
                MoveRadians(joints);
                return;
            }
            
            if (!Academy.Instance.IsCommunicatorOn)
            {
                SnapRadians(joints);
                return;
            }

            Evaluate(joints);
        }

        public override void Heuristic(in ActionBuffers actionsOut)
        {
        }

        private void Evaluate(List<float> joints)
        {
            SnapRadians(joints);
            PhysicsStep();
        
            const float accuracyValue = 100;
            const float timeValue = 10;

            float accuracy = Accuracy(LastJoint.position, _mlAgentsPos, Root.transform.rotation, LastJoint.rotation, _mlAgentsRot);

            bool meetsRepeatability = accuracy <= repeatability;

            accuracy = (1 - accuracy) * accuracyValue;
            
            if (meetsRepeatability)
            {
                float maxTime = float.MinValue;
                for (int i = 0; i < _middle.Count; i++)
                {
                    float distance = math.abs(_middle[i] - joints[i]);
                    float time = distance / _maxSpeeds[i];
                    if (time > maxTime)
                    {
                        maxTime = time;
                    }
                }

                accuracy += (maxTime / _maxTime) * timeValue;
            }
            
            SetReward(accuracy);
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
            _mlAgentsPos = position;
            _mlAgentsRot = rotation;
            _move = true;
            RequestDecision();
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
            _mlAgentsPos = position;
            _mlAgentsRot = rotation;
            _move = false;
            RequestDecision();
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

        public void MoveMiddle()
        {
            MoveRadians(_middle);
        }

        public void SnapMiddle()
        {
            SnapRadians(_middle);
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
            
            foreach (RobotJoint j in _joints)
            {
                if (!j.HasMotion)
                {
                    continue;
                }

                if (j.XMotion)
                {
                    speeds.Add(j.SpeedX);
                }

                if (j.YMotion)
                {
                    speeds.Add(j.SpeedY);
                }

                if (j.ZMotion)
                {
                    speeds.Add(j.SpeedZ);
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

        private List<float> JointsScaled(List<float> joints)
        {
            for (int i = 0; i < joints.Count; i++)
            {
                joints[i] = math.clamp(joints[i] * (_limits[i].upper - _limits[i].lower) + _limits[i].lower, _limits[i].lower, _limits[i].upper);
            }

            return joints;
        }

        private Vector3 RelativePosition(Vector3 position) => Root.transform.InverseTransformPoint(position) / _chainLength;

        private Quaternion RelativeRotation(Quaternion rotation) => Quaternion.Inverse(Root.transform.rotation) * rotation;

        public List<float> BioIkOptimize(Vector3 position, Quaternion orientation)
        {
            List<float> joints = GetJoints();
            
            List<float> best = new();
            best.AddRange(joints);
            
            float[] maxSpeeds = new float[joints.Count];
            for (int i = 0; i < joints.Count; i++)
            {
                maxSpeeds[i] = _maxSpeeds[i];
            }

            float bestAccuracy = float.MaxValue;
            float bestTime = float.MaxValue;

            for (int attempt = 0; attempt < optimizeAttempts; attempt++)
            {
                List<float> solution = BioIkSolve(position, orientation, joints);

                ProcessMotion();

                float accuracy = Accuracy(_lastBioIkJoint.position, position, Root.transform.rotation, _lastBioIkJoint.rotation, orientation);
                float time = CalculateTime(joints, solution, maxSpeeds);

                if (bestAccuracy > repeatability && accuracy < bestAccuracy)
                {
                    best = solution;
                    bestAccuracy = accuracy;
                    bestTime = time;
                    continue;
                }

                if (accuracy <= repeatability && time < bestTime)
                {
                    best = solution;
                    bestAccuracy = accuracy;
                    bestTime = time;
                }
            }

            return best;
        }
        
        public List<float> BioIkSolve(Vector3 position, Quaternion orientation)
        {
            return BioIkSolve(position, orientation, GetJoints());
        }

        private List<float> BioIkSolve(Vector3 position, Quaternion orientation, IReadOnlyList<float> starting)
        {
            double[] doubles = new double[starting.Count];
            for (int i = 0; i < starting.Count; i++)
            {
                doubles[i] = starting[i];
                _motions[i].SetTargetValue(doubles[i]);
            }
            doubles = new BioIkEvolution(this).Optimise(generations, doubles, position, orientation);
            return doubles.Select(t => (float) t).ToList();
        }

        private static float Accuracy(Vector3 currentPosition, Vector3 goalPosition, Quaternion rootRotation, Quaternion currentEndRotation, Quaternion goalEndRotation)
        {
            return Vector3.Distance(currentPosition, goalPosition) + Quaternion.Angle(goalEndRotation, Quaternion.Inverse(rootRotation) * currentEndRotation);
        }
        
        private static float CalculateTime(IEnumerable<float> starting, IReadOnlyList<float> ending, IReadOnlyList<float> maxSpeeds)
        {
            return starting.Select((t, i) => Math.Abs(t - ending[i]) / maxSpeeds[i]).Prepend(0).Max();
        }

        private void SetRandomJoints()
        {
            List<float> joints = new();
            for (int i = 0; i < _limits.Length; i++)
            {
                joints.Add(Random.Range(_limits[i].lower, _limits[i].upper));
            }

            SnapRadians(joints);
        }
        
        private List<float> NetScaledJoints()
        {
            List<float> joints = GetJoints();
            for (int i = 0; i < joints.Count; i++)
            {
                joints[i] = (joints[i] - _limits[i].lower) / (_limits[i].upper - _limits[i].lower);
            }

            return joints;
        }

        private void UpdateData()
        {
            foreach (BioIkJoint j in BioIkJoints)
            {
                j.UpdateData();
            }
        }

        private void ProcessMotion()
        {
            foreach (BioIkJoint j in BioIkJoints)
            {
                j.ProcessMotion();
            }
        }

        private static void PhysicsStep()
        {
            Physics.autoSimulation = false;
            Physics.Simulate(1);
            Physics.autoSimulation = true;
        }
    }
}