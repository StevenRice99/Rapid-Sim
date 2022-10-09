using System.Linq;
using UnityEngine;

[RequireComponent(typeof(RobotAgent))]
public class RobotForwardKinematicsTester : RobotHomeTester
{
    [SerializeField]
    private float[] angles;

    protected override void Update()
    {
        if (move)
        {
            move = false;
            Robot.Move(angles.ToList());
        }

        if (snap)
        {
            snap = false;
            Robot.Snap(angles.ToList());
        }
    }
}