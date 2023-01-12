using UnityEngine;

namespace RapidSim.Testers
{
    public class BioIkOptimizerTester : RobotHomeTester
    {
        protected override void Move()
        {
            Transform t = transform;
            robot.MoveRadians(robot.BioIkOptimize(t.position, t.rotation));
        }

        protected override void Snap()
        {
            Transform t = transform;
            robot.SnapRadians(robot.BioIkOptimize(t.position, t.rotation));
        }
    }
}