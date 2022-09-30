using System.Collections.Generic;
using System.Linq;
using Unity.MLAgents;
using UnityEngine;

public class RobotAgent : Agent
{
    private ArticulationBody _root;

    private void Awake()
    {
        _root = GetComponent<ArticulationBody>();
        if (_root == null)
        {
            _root = GetComponentInChildren<ArticulationBody>();
        }
    }

    public void MoveJoints(IEnumerable<float> degrees)
    {
        MoveJointsRadians(DegreesToRadians(degrees));
    }

    public void MoveJointsRadians(List<float> radians)
    {
        _root.SetDriveTargets(radians);
    }
    
    public void SetJoints(IEnumerable<float> degrees)
    {
        SetJointsRadians(DegreesToRadians(degrees));
    }

    public void SetJointsRadians(List<float> radians)
    {
        MoveJointsRadians(radians);
        _root.SetJointPositions(radians);
    }

    private static List<float> DegreesToRadians(IEnumerable<float> degrees)
    {
        List<float> radians = degrees.ToList();
        for (int i = 0; i < radians.Count; i++)
        {
            radians[i] *= Mathf.Deg2Rad;
        }

        return radians;
    }
}