using UnityEngine;

namespace RapidSim.Testers
{
    public class BioIkSolverTester : RobotHomeTester
    {
        protected override void Move()
        {
            Transform t = transform;
            robot.MoveRadians(robot.BioIkSolve(t.position, t.rotation));
        }

        protected override void Snap()
        {
            Transform t = transform;
            robot.SnapRadians(robot.BioIkSolve(t.position, t.rotation));
        }
    }
}