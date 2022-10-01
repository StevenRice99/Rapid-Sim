using System.Linq;
using UnityEngine;

public class RobotMoveTester : RobotHomeTester
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