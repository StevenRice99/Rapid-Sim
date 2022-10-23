using System.Collections.Generic;
using UnityEngine;
using Random = UnityEngine.Random;

[DisallowMultipleComponent]
[RequireComponent(typeof(RobotSolver))]
public class RobotTrainer : MonoBehaviour
{
    public RobotController _robotController { get; private set; }

    public RobotSolver _robotSolver { get; private set; }
    
    private void Start()
    {
        _robotController = GetComponent<RobotController>();
        _robotSolver = GetComponent<RobotSolver>();
    }

    private void OnEnable()
    {
        RobotTrainerManager.Register(this);
    }

    private void OnDisable()
    {
        RobotTrainerManager.Unregister(this);
    }

    public IEnumerable<float> RandomOrientation()
    {
        float[] randomAngles = new float[_robotController.LowerLimits.Length];
        for (int i = 0; i < _robotController.LowerLimits.Length; i++)
        {
            randomAngles[i] = Random.Range(_robotController.LowerLimits[i], _robotController.UpperLimits[i]);
        }

        return randomAngles;
    }
}