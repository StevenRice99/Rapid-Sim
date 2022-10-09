using System;
using System.Collections.Generic;
using System.Linq;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;
using Unity.MLAgents.Sensors;
using UnityEngine;
using Random = UnityEngine.Random;

[DisallowMultipleComponent]
public class RobotAgent : Agent
{
    [SerializeField]
    private float[] maxSpeeds;

    private ArticulationBody _root;

    private Transform _lastJoint;

    private List<float> _home;

    private List<float> _zeros;

    private List<float> _targets;

    private bool _move;

    private bool _canSolve;

    private float[] _currentSpeeds;

    private Vector3 _goalPosition;

    private Quaternion _goalRotation;

    private float[] _lowerLimits;

    private float[] _upperLimits;

    private List<float> _startAngles;

    private Vector3 _startPosition;

    private Quaternion _startRotation;

    private float _chainLength;

    public override void Initialize()
    {
        _root = GetComponent<ArticulationBody>();
        
        ArticulationBody[] children = GetComponentsInChildren<ArticulationBody>();

        if (_root == null)
        {
            if (children.Length == 0)
            {
                Debug.LogError($"No articulation bodies attached to {name}.");
            }
            else
            {
                _root = children[0];
            }
        }
        else if (children.Length > 0)
        {
            _chainLength = Vector3.Distance(_root.transform.position, children[0].transform.position);
        }

        _home = GetJoints();
        _lowerLimits = new float[_home.Count];
        _upperLimits = new float[_home.Count];

        ArticulationBody last = _root;

        int dofIndex = 0;
        
        for (int i = 0; i < children.Length; i++)
        {
            if (children[i].index > last.index)
            {
                last = children[i];
            }

            dofIndex = GetLimits(children[i].xDrive, dofIndex);
            dofIndex = GetLimits(children[i].yDrive, dofIndex);
            dofIndex = GetLimits(children[i].zDrive, dofIndex);

            if (i == 0)
            {
                continue;
            }
            
            _chainLength += Vector3.Distance(children[i].transform.position, children[i - 1].transform.position);
        }

        if (dofIndex != _lowerLimits.Length)
        {
            Debug.LogError($"Ensure all joints on {name} have limits defined.");
        }

        if (last != null)
        {
            _lastJoint = last.transform;
        }
        
        _zeros = new();
        for (int i = 0; i < _home.Count; i++)
        {
            _zeros.Add(0);
        }

        SetMaxSpeeds(maxSpeeds);
        _currentSpeeds = new float[maxSpeeds.Length];
        for (int i = 0; i < _currentSpeeds.Length; i++)
        {
            _currentSpeeds[i] = maxSpeeds[i];
        }

        if (_home.Count != maxSpeeds.Length)
        {
            Debug.LogError($"{name} has {_home.Count} degrees of freedom but {maxSpeeds.Length} speeds defined.");
        }

        BehaviorParameters parameters = GetComponent<BehaviorParameters>();
        if (parameters == null)
        {
            parameters = gameObject.AddComponent<BehaviorParameters>();
        }

        MaxStep = Academy.Instance.IsCommunicatorOn ? 1 : 0;

        parameters.BrainParameters.VectorObservationSize = 7 + _home.Count;
        parameters.BrainParameters.NumStackedVectorObservations = 1;
        ActionSpec spec = parameters.BrainParameters.ActionSpec;
        spec.NumContinuousActions = _home.Count;
        spec.BranchSizes = Array.Empty<int>();
        parameters.BrainParameters.ActionSpec = spec;
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
        Move(position, _goalRotation);
    }

    public void Move(Quaternion rotation)
    {
        Move(_goalPosition, rotation);
    }

    public void Move(Vector3 position, Quaternion rotation)
    {
        SetGoals(position, rotation);
        _move = true;
        _canSolve = false;
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
            angles[i] = Mathf.Abs(angles[i] - _targets[i]);
            if (angles[i] / maxSpeeds[i] > time)
            {
                time = angles[i] / maxSpeeds[i];
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
        _canSolve = true;
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
        Snap(position, _goalRotation);
    }

    public void Snap(Quaternion rotation)
    {
        Snap(_goalPosition, rotation);
    }
    
    public void Snap(Vector3 position, Quaternion rotation)
    {
        SetGoals(position, rotation);
        _move = false;
        _canSolve = false;
        RequestDecision();
    }
    
    public void Snap(List<float> degrees)
    {
        SnapRadians(DegreesToRadians(degrees));
    }

    public void SnapRadians(List<float> radians)
    {
        _move = false;
        _canSolve = false;
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

    public void SetMaxSpeeds(IEnumerable<float> degrees)
    {
        SetMaxSpeedsRadians(DegreesToRadians(degrees.ToList()).ToArray());
    }

    public void SetMaxSpeedsRadians(float[] radians)
    {
        maxSpeeds = radians;
    }

    public List<float> GetJoints()
    {
        List<float> angles = new();
        _root.GetJointPositions(angles);
        return angles;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(_goalPosition / _chainLength);
        sensor.AddObservation(_goalRotation);
        sensor.AddObservation(ScaleJoints());
    }

    public override void OnEpisodeBegin()
    {
        if (!Academy.Instance.IsCommunicatorOn)
        {
            return;
        }
        
        Randomize();
        RequestDecision();
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        List<float> joints = actions.ContinuousActions.ToList();
        for (int i = 0; i < joints.Count; i++)
        {
            joints[i] = Mathf.Clamp(joints[i] * (_upperLimits[i] - _lowerLimits[i]) + _lowerLimits[i], _lowerLimits[i], _upperLimits[i]);
        }

        _canSolve = true;
        
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

    private void FixedUpdate()
    {
        if (!_move || !_canSolve)
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
            _root.SetDriveTargets(delta);
        }
        else
        {
            Stop(delta);
        }
    }

    private void Stop(IEnumerable<float> radians)
    {
        List<float> list = radians.ToList();
        _root.SetDriveTargets(list);
        _root.SetJointVelocities(_zeros);
        _root.SetJointAccelerations(_zeros);
        _root.SetJointForces(_zeros);
        _root.SetJointPositions(list);
    }

    private void SetGoals(Vector3 position, Quaternion rotation)
    {
        _goalPosition = RelativePosition(position);
        _goalRotation = RelativeRotation(rotation);
    }

    private void Randomize()
    {
        Physics.autoSimulation = false;
        RandomGoal();
        RandomStart();
        Physics.autoSimulation = true;
    }

    private void RandomStart()
    {
        SnapRadians(RandomOrientation().ToList());
        Physics.Simulate(Time.fixedDeltaTime);
        _startPosition = RelativePosition(_lastJoint.position);
        _startRotation = RelativeRotation(_lastJoint.rotation);
        _startAngles = GetJoints();
    }

    private void RandomGoal()
    {
        SnapRadians(RandomOrientation().ToList());
        Physics.Simulate(Time.fixedDeltaTime);
        SetGoals(_lastJoint.position, _lastJoint.rotation);
        _targets = GetJoints();
    }

    private IEnumerable<float> RandomOrientation()
    {
        float[] randomAngles = new float[_lowerLimits.Length];
        for (int i = 0; i < _lowerLimits.Length; i++)
        {
            randomAngles[i] = Random.Range(_lowerLimits[i], _upperLimits[i]);
        }

        return randomAngles;
    }

    private int GetLimits(ArticulationDrive drive, int dofIndex)
    {
        if (drive.lowerLimit == 0 && drive.upperLimit == 0)
        {
            return dofIndex;
        }

        _lowerLimits[dofIndex] = drive.lowerLimit * Mathf.Deg2Rad;
        _upperLimits[dofIndex] = drive.upperLimit * Mathf.Deg2Rad;
        return ++dofIndex;
    }

    private void Evaluate(List<float> joints)
    {
        Physics.autoSimulation = false;
        SnapRadians(joints);
        Physics.Simulate(Time.fixedDeltaTime);
        Physics.autoSimulation = false;
        
        const float positionValue = 100;
        const float rotationValue = 100;
        const float timeValue = 1;

        float total = 0;

        float startingDistance = Vector3.Distance(_goalPosition, _startPosition);
        float score;
        float ending = Vector3.Distance(_goalPosition, RelativePosition(_lastJoint.position));
        if (ending != 0)
        {
            score = (startingDistance - ending) / startingDistance * positionValue;
            if (float.IsNaN(score))
            {
                score = positionValue;
            }
            
            SetReward(score);
            return;
        }
        
        total += positionValue * 2;
        
        ending = Quaternion.Angle(_goalRotation, RelativeRotation(_lastJoint.rotation));
        if (ending != 0)
        {
            float starting = Quaternion.Angle(_goalRotation, _startRotation);
            score = (starting - ending) / starting * rotationValue;
            if (float.IsNaN(score))
            {
                score = rotationValue;
            }
            total += score;
            
            SetReward(total);
            return;
        }

        total += rotationValue * 2;

        float time = 0;
        for (int i = 0; i < _startAngles.Count; i++)
        {
            _startAngles[i] = Mathf.Abs(_startAngles[i] - _targets[i]);
            if (_startAngles[i] / maxSpeeds[i] > time)
            {
                time = _startAngles[i] / maxSpeeds[i];
            }
        }
        score = -(time / startingDistance) * timeValue;
        if (float.IsNaN(score))
        {
            score = 0;
        }
        total += score;
        
        SetReward(total);
    }
    
    private Vector3 RelativePosition(Vector3 position) => _root.transform.InverseTransformPoint(position);
    
    private Quaternion RelativeRotation(Quaternion rotation) => Quaternion.Inverse(_root.transform.rotation) * rotation;

    private List<float> ScaleJoints()
    {
        List<float> joints = GetJoints();
        for (int i = 0; i < joints.Count; i++)
        {
            joints[i] = (joints[i] - _lowerLimits[i]) / (_upperLimits[i] - _lowerLimits[i]);
        }

        return joints;
    }

    private static List<float> DegreesToRadians(List<float> degrees)
    {
        for (int i = 0; i < degrees.Count; i++)
        {
            degrees[i] *= Mathf.Deg2Rad;
        }

        return degrees;
    }
}