using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class RobotTester : MonoBehaviour
{
    [SerializeField]
    private bool reset;
    
    [SerializeField]
    private bool move;

    [SerializeField]
    private bool snap;
    
    [SerializeField]
    private float[] angles;

    private RobotAgent _robotAgent;

    private void Awake()
    {
        _robotAgent = GetComponent<RobotAgent>();
    }

    private void Update()
    {
        if (reset)
        {
            reset = false;
            _robotAgent.SnapHome();
        }
        
        if (move)
        {
            move = false;
            _robotAgent.MoveJoints(angles.ToList());
        }

        if (snap)
        {
            snap = false;
            _robotAgent.SetJoints(angles.ToList());
        }
    }
}