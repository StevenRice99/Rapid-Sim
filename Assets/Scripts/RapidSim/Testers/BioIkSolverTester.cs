using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace RapidSim.Testers
{
    public class BioIkSolverTester : RobotHomeTester
    {
        protected override void Move()
        {
            Transform t = transform;
            List<double> doubles = robot.BioIkSolve(t.position, t.rotation).ToList();
            List<float> joints = new();
            for (int i = 0; i < doubles.Count; i++)
            {
                joints.Add((float) doubles[i]);
            }
            
            robot.MoveRadians(joints);
        }

        protected override void Snap()
        {
            Transform t = transform;
            List<double> doubles = robot.BioIkSolve(t.position, t.rotation).ToList();
            List<float> joints = new();
            for (int i = 0; i < doubles.Count; i++)
            {
                joints.Add((float) doubles[i]);
            }
            
            robot.SnapRadians(joints);
        }
    }
}