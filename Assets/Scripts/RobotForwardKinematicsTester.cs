using System.Linq;
using UnityEngine;

public class RobotForwardKinematicsTester : RobotHomeTester
{
    [SerializeField]
    private float[] angles;

    protected override void Move()
    {
        robot.Move(angles.ToList());
    }

    protected override void Snap()
    {
        robot.Snap(angles.ToList());
    }
}