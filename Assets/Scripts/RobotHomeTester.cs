using System;
using UnityEngine;

public class RobotHomeTester : MonoBehaviour
{
    [SerializeField]
    protected RobotAgent robot;
    
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
        
        robot = GetComponent<RobotAgent>();
        if (robot != null)
        {
            return;
        }
        
        robot = GetComponentInChildren<RobotAgent>();
        if (robot != null)
        {
            return;
        }

        robot = FindObjectOfType<RobotAgent>();
    }

    private void OnValidate()
    {
        Awake();
    }

    protected virtual void Update()
    {
        if (move)
        {
            move = false;
            robot.MoveHome();
        }

        if (snap)
        {
            snap = false;
            robot.SnapHome();
        }
    }
}