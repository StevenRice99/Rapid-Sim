using System.Collections.Generic;
using UnityEngine;
using Random = UnityEngine.Random;

[DisallowMultipleComponent]
[RequireComponent(typeof(RobotSolver))]
public class RobotTrainer : MonoBehaviour
{
    public RobotController RobotController { get; private set; }

    public RobotSolver RobotSolver { get; private set; }
    
    private void Start()
    {
        RobotController = GetComponent<RobotController>();
        RobotSolver = GetComponent<RobotSolver>();
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
        float[] randomAngles = new float[RobotController.LowerLimits.Length];
        for (int i = 0; i < RobotController.LowerLimits.Length; i++)
        {
            randomAngles[i] = Random.Range(RobotController.LowerLimits[i], RobotController.UpperLimits[i]);
        }

        return randomAngles;
    }
}