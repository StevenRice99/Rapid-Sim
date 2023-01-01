using System.Linq;
using UnityEngine;

namespace RapidSim.Testers
{
    public class RobotForwardKinematicsTester : RobotHomeTester
    {
        [Tooltip("The values to move the robot to.")]
        [SerializeField]
        private float[] values;

        protected override void Move()
        {
            robot.Move(values.ToList());
        }

        protected override void Snap()
        {
            robot.Snap(values.ToList());
        }
    }
}