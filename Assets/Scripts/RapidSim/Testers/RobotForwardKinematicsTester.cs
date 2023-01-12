using System.Collections.Generic;
using UnityEngine;

namespace RapidSim.Testers
{
    public class RobotForwardKinematicsTester : RobotHomeTester
    {
        [Tooltip("The values to move the robot to.")]
        [SerializeField]
        private List<float> values;

        [Tooltip("If values are in radians or not.")]
        [SerializeField]
        private bool radians;

        protected override void Move()
        {
            if (radians)
            {
                robot.MoveRadians(values);
            }
            else
            {
                robot.Move(values);
            }
        }

        protected override void Snap()
        {
            if (radians)
            {
                robot.SnapRadians(values);
            }
            else
            {
                robot.Snap(values);
            }
        }
    }
}