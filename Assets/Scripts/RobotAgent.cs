using System.Collections.Generic;
using Unity.MLAgents;
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

        List<float> home = new();
        _root.GetJointPositions(home);
        _home = home;
        
        _zeros = new();
        for (int i = 0; i < _home.Count; i++)
        {
            _zeros.Add(0);
        }

        if (_home.Count != maxSpeeds.Length)
        {
            Debug.LogError($"{name} has {_home.Count} degrees of freedom but {maxSpeeds.Length} speeds defined.");
        }
    }

    public void Move(List<float> degrees)
    {
        MoveRadians(DegreesToRadians(degrees));
    }

    public void MoveRadians(List<float> radians)
    {
        _targets = radians;
        _move = true;
    }
    
    public void Snap(List<float> degrees)
    {
        SnapRadians(DegreesToRadians(degrees));
    }

    public void SnapRadians(List<float> radians)
    {
        _move = false;
        _root.SetDriveTargets(radians);
        _root.SetJointVelocities(_zeros);
        _root.SetJointAccelerations(_zeros);
        _root.SetJointForces(_zeros);
        _root.SetJointPositions(radians);
    }

    public void MoveHome()
    {
        MoveRadians(_home);
    }

    public void SnapHome()
    {
        SnapRadians(_home);
    }

    private void FixedUpdate()
    {
        if (!_move)
        {
            return;
        }

        _move = false;

        List<float> delta = new();
        _root.GetJointPositions(delta);
        for (int i = 0; i < delta.Count; i++)
        {
            if (delta[i] >= _targets[i])
            {
                delta[i] -= maxSpeeds[i] * Mathf.Deg2Rad * Time.fixedDeltaTime;
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
                delta[i] += maxSpeeds[i] * Mathf.Deg2Rad * Time.fixedDeltaTime;
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
        
        _root.SetDriveTargets(delta);
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