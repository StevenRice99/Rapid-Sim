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
        public BioIkJoint[] BioIkJoints { get; private set; }
        
        public float Rescaling { get; private set; }

        public int PopulationSize => robotProperties.PopulationSize;

        public int Elites => robotProperties.Elites;

        [Tooltip("The robot properties to use.")]
        [SerializeField]
        private RobotProperties robotProperties;

        private ArticulationBody Root => _joints[0].Joint;

        private Transform LastJoint => _joints[^1].transform;

        private List<float> _home;

        private List<float> _zeros;

        private List<float> _targets;

        private bool _move;

        private float[] _maxSpeeds;

        private float[] _currentSpeeds;

        private JointLimit[] _limits;

        private RobotJoint[] _joints;

        private BioIkJoint.Motion[] _motions;

        private float _chainLength;

        private Vector3 _mlAgentsPos;

        private Quaternion _mlAgentsRot;

        private List<float> _mlAgentsJoints;

        private float _maxTime;

        public void Start()
        {
            if (robotProperties == null)
            {
                Debug.LogError($"No robot properties attached to {name}.");
                Destroy(gameObject);
                return;
            }
            
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

            Rescaling = math.PI * math.PI / (_chainLength * _chainLength);

            _limits = limits.ToArray();
            _home = GetJoints();
            
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

            foreach (BioIkJoint j in BioIkJoints)
            {
                j.UpdateData();
            }
            
            BehaviorParameters parameters = GetComponent<BehaviorParameters>();
            if (parameters == null)
            {
                parameters = gameObject.AddComponent<BehaviorParameters>();
            }

            MaxStep = Academy.Instance.IsCommunicatorOn ? 1 : 0;

            parameters.BehaviorName = robotProperties.name;
            parameters.BrainParameters.VectorObservationSize = 7 + _limits.Length;
            parameters.BrainParameters.NumStackedVectorObservations = 1;
            ActionSpec spec = parameters.BrainParameters.ActionSpec;
            spec.NumContinuousActions = _home.Count;
            spec.BranchSizes = Array.Empty<int>();
            parameters.BrainParameters.ActionSpec = spec;
            parameters.DeterministicInference = true;
            parameters.TeamId = 0;
            parameters.UseChildSensors = false;
            parameters.UseChildActuators = false;
            parameters.ObservableAttributeHandling = ObservableAttributeOptions.Ignore;
        }

        public override void OnEpisodeBegin()
        {
            if (!Academy.Instance.IsCommunicatorOn)
            {
                return;
            }

            bool move = _move;
            
            List<float> joints = new();
            for (int i = 0; i < _limits.Length; i++)
            {
                joints.Add(Random.Range(_limits[i].lower, _limits[i].upper));
            }
            SnapRadians(joints);
            PhysicsStep();
            
            _move = move;
            
            _mlAgentsPos = Random.insideUnitSphere * _chainLength + transform.position;
            _mlAgentsRot = Random.rotation;
            _mlAgentsJoints = GetJoints();
            
            RequestDecision();
        }

        public override void CollectObservations(VectorSensor sensor)
        {
            sensor.AddObservation(RelativePosition(_mlAgentsPos));
            sensor.AddObservation(RelativeRotation(_mlAgentsRot));
            sensor.AddObservation(NetScaledJoints(_mlAgentsJoints));
        }

        public override void OnActionReceived(ActionBuffers actions)
        {
            List<float> joints = actions.ContinuousActions.ToList();
            
            for (int i = 0; i < joints.Count; i++)
            {
                joints[i] = math.clamp(joints[i] * (_limits[i].upper - _limits[i].lower) + _limits[i].lower, _limits[i].lower, _limits[i].upper);
            }

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
            List<float> solution = NetScaledJoints(BioIkOptimize(_mlAgentsPos, _mlAgentsRot));
            
            ActionSegment<float> continuous = actionsOut.ContinuousActions;

            for (int i = 0; i < solution.Count; i++)
            {
                continuous[i] = solution[i];
            }
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
            _mlAgentsJoints = GetJoints();
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
            _mlAgentsJoints = GetJoints();
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

        public List<float> BioIkOptimize(Vector3 position, Quaternion orientation)
        {
            List<float> starting = GetJoints();
            
            List<float> best = new();
            best.AddRange(starting);

            float bestAccuracy = float.MaxValue;
            float bestTime = float.MaxValue;

            bool move = _move;

            for (int attempt = 0; attempt < robotProperties.OptimizeAttempts; attempt++)
            {
                SnapRadians(BioIkSolve(position, orientation, starting));
                PhysicsStep();

                List<float> solution = GetJoints();

                float accuracy = Accuracy(LastJoint.position, position, Root.transform.rotation, LastJoint.rotation, orientation);
                float time = CalculateTime(starting, solution, _maxSpeeds);

                if ((bestAccuracy <= robotProperties.Repeatability || accuracy >= bestAccuracy) && (accuracy > robotProperties.Repeatability || time >= bestTime))
                {
                    continue;
                }

                best = solution;
                bestAccuracy = accuracy;
                bestTime = time;
            }
            
            SnapRadians(starting);
            PhysicsStep();

            _move = move;

            return best;
        }
        
        public List<float> BioIkSolve(Vector3 position, Quaternion orientation)
        {
            return BioIkSolve(position, orientation, GetJoints());
        }

        private static void PhysicsStep()
        {
            Physics.autoSimulation = false;
            Physics.Simulate(1);
            Physics.autoSimulation = true;
        }

        private Vector3 RelativePosition(Vector3 position) => Root.transform.InverseTransformPoint(position) / _chainLength;

        private Quaternion RelativeRotation(Quaternion rotation) => Quaternion.Inverse(Root.transform.rotation) * rotation;

        private List<float> BioIkSolve(Vector3 position, Quaternion orientation, IReadOnlyList<float> starting)
        {
            double[] doubles = new double[starting.Count];
            for (int i = 0; i < starting.Count; i++)
            {
                doubles[i] = starting[i];
                _motions[i].SetTargetValue(doubles[i]);
            }
            doubles = new BioIkEvolution(this).Optimise(robotProperties.Generations, doubles, position, orientation);
            return doubles.Select(t => (float) t).ToList();
        }

        private void Evaluate(List<float> joints)
        {
            bool move = _move;
            SnapRadians(joints);
            PhysicsStep();
            _move = move;

            float accuracy = Accuracy(LastJoint.position, _mlAgentsPos, Root.transform.rotation, LastJoint.rotation, _mlAgentsRot);

            accuracy = accuracy <= robotProperties.Repeatability
                ? robotProperties.ValueAccuracy + CalculateTime(_mlAgentsJoints, joints, _maxSpeeds) / _maxTime * robotProperties.ValueTime
                : (1 - accuracy) * robotProperties.ValueAccuracy;

            SetReward(accuracy);
        }

        private List<float> GetJoints()
        {
            List<float> angles = new();
            Root.GetJointPositions(angles);
            return angles;
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

        private static float Accuracy(Vector3 currentPosition, Vector3 goalPosition, Quaternion rootRotation, Quaternion currentEndRotation, Quaternion goalEndRotation)
        {
            return Vector3.Distance(currentPosition, goalPosition) + Quaternion.Angle(goalEndRotation, Quaternion.Inverse(rootRotation) * currentEndRotation);
        }
        
        private static float CalculateTime(IEnumerable<float> starting, IReadOnlyList<float> ending, IReadOnlyList<float> maxSpeeds)
        {
            return starting.Select((t, i) => Math.Abs(t - ending[i]) / maxSpeeds[i]).Prepend(0).Max();
        }
        
        private List<float> NetScaledJoints(List<float> joints)
        {
            for (int i = 0; i < joints.Count; i++)
            {
                joints[i] = (joints[i] - _limits[i].lower) / (_limits[i].upper - _limits[i].lower);
            }

            return joints;
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
    }
}