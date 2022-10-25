using UnityEngine;

namespace RapidSim.Testers
{
    public class RobotHomeTester : RobotTester
    {
        [SerializeField]
        protected RobotController robot;

        protected override void Awake()
        {
            if (robot != null)
            {
                return;
            }
        
            robot = GetComponent<RobotController>();
            if (robot != null)
            {
                return;
            }
        
            robot = GetComponentInChildren<RobotController>();
            if (robot != null)
            {
                return;
            }

            robot = FindObjectOfType<RobotController>();
        }

        protected override void Move()
        {
            robot.MoveHome();
        }

        protected override void Snap()
        {
            robot.SnapHome();
        }
    }
}