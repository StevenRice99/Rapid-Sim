using UnityEngine;

public class RobotTester : MonoBehaviour
{
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
        if (move)
        {
            move = false;
            _robotAgent.MoveJoints(angles);
        }

        if (snap)
        {
            snap = false;
            _robotAgent.SetJoints(angles);
        }
    }
}