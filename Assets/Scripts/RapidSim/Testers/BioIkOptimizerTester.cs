using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace RapidSim.Testers
{
    public class BioIkOptimizerTester : RobotHomeTester
    {
        protected override void Move()
        {
            Transform t = transform;
            double[] doubles = robot.BioIkOptimize(t.position, t.rotation);
            List<float> joints = new();
            for (int i = 0; i < doubles.Length; i++)
            {
                joints.Add((float) doubles[i]);
            }
            
            robot.MoveRadians(joints);
        }

        protected override void Snap()
        {
            Transform t = transform;
            double[] doubles = robot.BioIkOptimize(t.position, t.rotation);
            List<float> joints = new();
            for (int i = 0; i < doubles.Length; i++)
            {
                joints.Add((float) doubles[i]);
            }
            
            robot.SnapRadians(joints);
        }
    }
}