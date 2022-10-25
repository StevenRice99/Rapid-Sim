using System.Collections.Generic;
using System.Linq;
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

        private readonly List<RobotTrainer> _trainers = new();

        public static void Register(RobotTrainer robotTrainer)
        {
            if (Singleton._trainers.Contains(robotTrainer))
            {
                return;
            }
        
            Singleton._trainers.Add(robotTrainer);
        }

        public static void Unregister(RobotTrainer robotTrainer)
        {
            Singleton._trainers.Remove(robotTrainer);
            if (Singleton._trainers.Count > 0)
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
            for (int i = 0; i < _trainers.Count; i++)
            {
                _trainers[i].RobotController.SnapRadians(_trainers[i].RandomOrientation().ToList());
            }

            Physics.Simulate(Time.fixedDeltaTime);

            List<float>[] startAngles = new List<float>[_trainers.Count];
        
            for (int i = 0; i < _trainers.Count; i++)
            {
                startAngles[i] = _trainers[i].RobotController.GetJoints();
                _trainers[i].RobotController.SnapRadians(_trainers[i].RandomOrientation().ToList());
            }

            Physics.Simulate(Time.fixedDeltaTime);

            for (int i = 0; i < _trainers.Count; i++)
            {
                Vector3 goalPosition = _trainers[i].RobotController.LastJoint.position;
                Quaternion goalRotation = _trainers[i].RobotController.LastJoint.rotation;
        
                // TODO: Bio IK solve, get goal joint values.
                // TODO: Convert results to radians.
                float[] expected = new float[startAngles[i].Count];
                
                _trainers[i].Train(goalPosition, goalRotation, startAngles[i], expected);
            }
        }
    }
}