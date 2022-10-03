using System;
using System.Collections.Generic;
using System.Linq;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;
using Unity.MLAgents.Sensors;
using UnityEngine;
using Random = UnityEngine.Random;

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

    private float[] _currentSpeeds;

    private Vector3 _goalPosition;

    private Quaternion _goalRotation;

    private float[] _lowerLimits;

    private float[] _upperLimits;

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

        MaxStep = 1;

        BehaviorParameters parameters = GetComponent<BehaviorParameters>();
        if (parameters == null)
        {
            parameters = gameObject.AddComponent<BehaviorParameters>();
        }

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
        int slowest = -1;
        float time = 0;
        for (int i = 0; i < angles.Count; i++)
        {
            angles[i] = Mathf.Abs(angles[i] - _targets[i]);
            if (slowest >= 0 && angles[i] / maxSpeeds[i] <= time)
            {
                continue;
            }

            slowest = i;
            time = angles[i] / maxSpeeds[i];
        }

        for (int i = 0; i < _currentSpeeds.Length; i++)
        {
            _currentSpeeds[i] = angles[i] / time;
        }
        
        _move = true;
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
        sensor.AddObservation(_goalPosition);
        sensor.AddObservation(_goalRotation);
        sensor.AddObservation(GetJoints());
    }

    public override void OnEpisodeBegin()
    {
        SnapRadians(RandomOrientation().ToList());
        SetGoals(_lastJoint.position, _lastJoint.rotation);
        SnapRadians(RandomOrientation().ToList());
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        if (_move)
        {
            MoveRadians(actions.ContinuousActions.ToList());
        }
        else
        {
            // ADD REWARDS
            SnapRadians(actions.ContinuousActions.ToList());
        }
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
            _root.SetDriveTargets(delta);
        }
        else
        {
            Stop(delta);
        }
    }

    private void Stop(List<float> radians)
    {
        _root.SetDriveTargets(radians);
        _root.SetJointVelocities(_zeros);
        _root.SetJointAccelerations(_zeros);
        _root.SetJointForces(_zeros);
        _root.SetJointPositions(radians);
    }

    private void SetGoals(Vector3 position, Quaternion rotation)
    {
        Transform rootTransform = _root.transform;
        _goalPosition = rootTransform.InverseTransformPoint(position);
        _goalRotation = Quaternion.Inverse(rootTransform.rotation) * rotation;
    }

    private IEnumerable<float> RandomOrientation()
    {
        float[] randomAngles = new float[_lowerLimits.Length];
        for (int i = 0; i < _lowerLimits.Length; i++)
        {
            randomAngles[i] = Random.Range(_lowerLimits[i], _upperLimits[i]) * Mathf.Deg2Rad;
        }

        return randomAngles;
    }

    private int GetLimits(ArticulationDrive drive, int dofIndex)
    {
        if (drive.lowerLimit == 0 && drive.upperLimit == 0)
        {
            return dofIndex;
        }

        _lowerLimits[dofIndex] = drive.lowerLimit;
        _upperLimits[dofIndex] = drive.upperLimit;
        return ++dofIndex;
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