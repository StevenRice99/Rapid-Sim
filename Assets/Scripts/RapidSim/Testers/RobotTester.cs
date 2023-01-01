using UnityEngine;

namespace RapidSim.Testers
{
    public abstract class RobotTester : MonoBehaviour
    {
        [Header("Controls")]
        [Tooltip("Move the robot over time.")]
        [SerializeField]
        protected bool move;

        [Tooltip("Instantly snap the robot.")]
        [SerializeField]
        protected bool snap;
        
        [Header("Configuration")]
        [Tooltip("The robot to control.")]
        [SerializeField]
        protected Robot robot;

        private void Awake()
        {
            if (robot != null)
            {
                return;
            }
        
            robot = GetComponent<Robot>();
            if (robot != null)
            {
                return;
            }
        
            robot = GetComponentInChildren<Robot>();
            if (robot != null)
            {
                return;
            }

            robot = FindObjectOfType<Robot>();
        }

        protected abstract void Move();

        protected abstract void Snap();

        private void OnValidate()
        {
            Awake();
        }
    
        private void Update()
        {
            if (move)
            {
                move = false;
                Move();
            }

            if (!snap)
            {
                return;
            }

            snap = false;
            Snap();
        }
    }
}