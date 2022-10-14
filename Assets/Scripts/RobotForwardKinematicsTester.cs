using System.Linq;
using UnityEngine;

public class RobotForwardKinematicsTester : RobotHomeTester
{
    [SerializeField]
    private float[] angles;

    protected override void Update()
    {
        if (move)
        {
            move = false;
            robot.Move(angles.ToList());
        }

        if (snap)
        {
            snap = false;
            robot.Snap(angles.ToList());
        }
    }
}