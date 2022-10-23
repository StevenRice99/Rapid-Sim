using UnityEngine;

public class RobotInverseKinematicsTester : RobotTester
{
    [SerializeField]
    protected RobotSolver robot;
    
    protected override void Awake()
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
    
    protected override void Move()
    {
        robot.Move(transform);
    }

    protected override void Snap()
    {
        robot.Snap(transform);
    }
}