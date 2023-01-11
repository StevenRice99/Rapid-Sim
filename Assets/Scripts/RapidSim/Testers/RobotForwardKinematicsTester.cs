using System.Linq;
using UnityEngine;

namespace RapidSim.Testers
{
    public class RobotForwardKinematicsTester : RobotHomeTester
    {
        [Tooltip("The values to move the robot to.")]
        [SerializeField]
        private float[] values;

        [Tooltip("If values are in radians or not.")]
        [SerializeField]
        private bool radians;

        protected override void Move()
        {
            if (radians)
            {
                robot.MoveRadians(values.ToList());
            }
            else
            {
                robot.Move(values.ToList());
            }
        }

        protected override void Snap()
        {
            if (radians)
            {
                robot.SnapRadians(values.ToList());
            }
            else
            {
                robot.Snap(values.ToList());
            }
        }
    }
}