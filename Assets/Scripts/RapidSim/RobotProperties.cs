using UnityEngine;

namespace RapidSim
{
    [CreateAssetMenu(fileName = "Robot Properties", menuName = "Rapid-Sim/Robot Properties", order = 0)]
    public class RobotProperties : ScriptableObject
    {
        public float Repeatability => repeatability;

        public int Generations => generations;

        public int Population => population;

        public int Elites => elites;

        public int Attempts => attempts;

        public float Accuracy => accuracy;
        
        public float Time => time;
        
        [Header("Movement")]
        [Tooltip("How accurate in meters the robot can repeat a movement.")]
        [Min(0)]
        [SerializeField]
        private float repeatability = 8e-5f;
        
        [Header("Bio IK")]
        [Tooltip("The number of generations for a Bio IK evolution.")]
        [Min(1)]
        [SerializeField]
        private int generations = 5;
        
        [Tooltip("The population size of each generation during Bio IK evolution.")]
        [Min(1)]
        [SerializeField]
        private int population = 120;
        
        [Tooltip("The number of elites in each generation during Bio IK evolution.")]
        [Min(1)]
        [SerializeField]
        private int elites = 3;
        
        [Tooltip("The number of times to run the Bio IK algorithm when attempting to find an optimal move during heuristics.")]
        [Min(1)]
        [SerializeField]
        private int attempts = 10;

        [Header("Reinforcement Learning")]
        [Tooltip("The value to give during reinforcement learning for achieving an accuracy within the required repeatability.")]
        [Min(0)]
        [SerializeField]
        private float accuracy = 100;
        
        [Tooltip("The value to give during reinforcement learning for a perfect (zero movement) time, meaning a perfect value will likely never happen. Only applies if repeatability is reached.")]
        [Min(0)]
        [SerializeField]
        private float time = 10;
        
        private void OnValidate()
        {
            if (elites > population)
            {
                elites = population;
            }
        }
    }
}