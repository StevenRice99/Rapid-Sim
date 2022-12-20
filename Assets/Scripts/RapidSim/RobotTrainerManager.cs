using System.Collections.Generic;
using UnityEngine;

namespace RapidSim
{
    public class RobotTrainerManager : MonoBehaviour
    {
        private static RobotTrainerManager Singleton
        {
            get
            {
                if (_singleton != null)
                {
                    return _singleton;
                }

                GameObject go = new("Robot Trainer Manager");
                RobotTrainerManager c = go.AddComponent<RobotTrainerManager>();
                _singleton = c;
                Physics.autoSimulation = false;
                return c;
            }
        }

        private static RobotTrainerManager _singleton;

        private readonly List<Robot> _robots = new();

        public static void Register(Robot robot)
        {
            if (Singleton._robots.Contains(robot))
            {
                return;
            }
        
            Singleton._robots.Add(robot);
        }

        public static void Unregister(Robot robot)
        {
            Singleton._robots.Remove(robot);
            if (Singleton._robots.Count > 0)
            {
                return;
            }

            Physics.autoSimulation = true;
            Destroy(Singleton.gameObject);
        }

        private void Awake()
        {
            if (_singleton != null)
            {
                if (_singleton != this)
                {
                    Destroy(gameObject);
                }
            
                return;
            }

            Physics.autoSimulation = false;
            _singleton = this;
        }

        private void Update()
        {
            double[][] starting = new double[_robots.Count][];
        
            for (int i = 0; i < _robots.Count; i++)
            {
                float[] angles = _robots[i].RandomOrientation();
                double[] start = new double[angles.Length];
                for (int j = 0; j < start.Length; j++)
                {
                    start[i] = angles[i];
                }

                starting[i] = start;
                _robots[i].SetRandomOrientation();
            }

            Physics.Simulate(Time.fixedDeltaTime);

            for (int i = 0; i < _robots.Count; i++)
            {
                Vector3 goalPosition = _robots[i].Objective.position;
                Quaternion goalRotation = _robots[i].Objective.rotation;
        
                double[] expected = _robots[i].NetScaled(_robots[i].BioIkSolve(goalPosition, goalRotation));

                /*
                Debug.Log("Expected Values:");
                for (int j = 0; j < expected.Length; j++)
                {
                    Debug.Log($"Expected {j}: {expected[j]}");
                }
                */

                _robots[i].Train(goalPosition, goalRotation, starting[i], expected);
            }
        }
    }
}