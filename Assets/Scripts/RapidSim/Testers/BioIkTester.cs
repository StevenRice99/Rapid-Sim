using System.Linq;
using UnityEngine;

namespace RapidSim.Testers
{
    public class BioIkTester : RobotHomeTester
    {
        [SerializeField]
        private RobotTrainer robotTrainer;
        
        protected override void Awake()
        {
            base.Awake();
            
            if (robotTrainer != null)
            {
                return;
            }
        
            robotTrainer = GetComponent<RobotTrainer>();
            if (robotTrainer != null)
            {
                return;
            }
        
            robotTrainer = GetComponentInChildren<RobotTrainer>();
            if (robotTrainer != null)
            {
                return;
            }

            robotTrainer = FindObjectOfType<RobotTrainer>();
        }

        protected override void Move()
        {
            Transform t = transform;
            robot.MoveRadians(robotTrainer.BioIkSolve(t.position, t.rotation).ToList());
        }

        protected override void Snap()
        {
            Transform t = transform;
            robot.SnapRadians(robotTrainer.BioIkSolve(t.position, t.rotation).ToList());
        }
    }
}