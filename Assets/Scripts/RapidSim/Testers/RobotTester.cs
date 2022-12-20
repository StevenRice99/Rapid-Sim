using UnityEngine;

namespace RapidSim.Testers
{
    public abstract class RobotTester : MonoBehaviour
    {
        [SerializeField]
        protected Robot robot;
        
        [SerializeField]
        protected bool move;

        [SerializeField]
        protected bool snap;

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