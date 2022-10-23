using UnityEngine;

public class RobotInverseKinematicsTester : MonoBehaviour
{
    [SerializeField]
    protected RobotSolver robot;
    
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
        
        robot = GetComponent<RobotSolver>();
        if (robot != null)
        {
            return;
        }
        
        robot = GetComponentInChildren<RobotSolver>();
        if (robot != null)
        {
            return;
        }

        robot = FindObjectOfType<RobotSolver>();
    }
    
    protected void Move()
    {
        robot.Move(transform);
    }

    protected void Snap()
    {
        robot.Snap(transform);
    }
}