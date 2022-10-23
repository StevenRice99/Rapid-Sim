using UnityEngine;

public class RobotHomeTester : MonoBehaviour
{
    [SerializeField]
    protected RobotController robot;
    
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
        
        robot = GetComponent<RobotController>();
        if (robot != null)
        {
            return;
        }
        
        robot = GetComponentInChildren<RobotController>();
        if (robot != null)
        {
            return;
        }

        robot = FindObjectOfType<RobotController>();
    }

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

    protected virtual void Move()
    {
        robot.MoveHome();
    }

    protected virtual void Snap()
    {
        robot.SnapHome();
    }
}