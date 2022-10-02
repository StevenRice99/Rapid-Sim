using System;
using System.Collections.Generic;
using System.Linq;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;
using Unity.MLAgents.Sensors;
using UnityEngine;

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

    private Transform _target;

    private void Start()
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

        ArticulationBody last = _root;
        for (int i = children.Length - 1; i > 0; i--)
        {
            if (children[i].index > last.index)
            {
                last = children[i];
            }
        }

        if (last != null)
        {
            _lastJoint = last.transform;
        }

        _home = GetJoints();
        
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

        _target = new GameObject("Target").transform;
        _target.parent = _root.transform;
        _target.position = _lastJoint.position;
        _target.rotation = _lastJoint.rotation;

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
        Move(position, _target.rotation);
    }

    public void Move(Quaternion rotation)
    {
        Move(_target.position, rotation);
    }

    public void Move(Vector3 position, Quaternion rotation)
    {
        _target.position = position;
        _target.rotation = rotation;
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
        sensor.AddObservation(_target.localPosition);
        sensor.AddObservation(_target.localRotation);
        sensor.AddObservation(GetJoints());
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

    private static List<float> DegreesToRadians(List<float> degrees)
    {
        for (int i = 0; i < degrees.Count; i++)
        {
            degrees[i] *= Mathf.Deg2Rad;
        }

        return degrees;
    }
}