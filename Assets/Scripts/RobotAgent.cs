using System.Collections.Generic;
using Unity.MLAgents;
using UnityEngine;

public class RobotAgent : Agent
{
    private ArticulationBody _root;

    private Transform _lastJoint;

    private List<float> _home;

    private void Awake()
    {
        _root = GetComponent<ArticulationBody>();
        
        ArticulationBody[] children = GetComponentsInChildren<ArticulationBody>();
        
        if (_root == null)
        {
            if (children.Length == 0)
            {
                Debug.LogError($"No Articulation Bodies attached to robot {name}.");
                Destroy(this);
                return;
            }
            
            _root = children[0];
        }

        ArticulationBody last = _root;
        for (int i = children.Length - 1; i > 0; i--)
        {
            if (children[i].index > last.index)
            {
                last = children[i];
            }
        }

        _lastJoint = last.transform;

        List<float> home = new();
        _root.GetJointPositions(home);
        _home = home;
    }

    public void MoveJoints(List<float> degrees)
    {
        MoveJointsRadians(DegreesToRadians(degrees));
    }

    public void MoveJointsRadians(List<float> radians)
    {
        _root.SetDriveTargets(radians);
    }
    
    public void SetJoints(List<float> degrees)
    {
        SetJointsRadians(DegreesToRadians(degrees));
    }

    public void SetJointsRadians(List<float> radians)
    {
        _root.SetDriveTargets(radians);
        _root.SetJointPositions(radians);
    }

    public void MoveHome()
    {
        _root.SetDriveTargets(_home);
    }

    public void SnapHome()
    {
        _root.SetDriveTargets(_home);
        _root.SetJointPositions(_home);
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