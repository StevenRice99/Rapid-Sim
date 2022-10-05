using UnityEngine;

[RequireComponent(typeof(RobotAgent))]
public class RobotHomeTester : MonoBehaviour
{
    [SerializeField]
    protected bool move;

    [SerializeField]
    protected bool snap;

    protected RobotAgent Robot;

    private void Awake()
    {
        Robot = GetComponent<RobotAgent>();
    }

    protected virtual void Update()
    {
        if (move)
        {
            move = false;
            Robot.MoveHome();
        }

        if (snap)
        {
            snap = false;
            Robot.SnapHome();
        }
    }
}