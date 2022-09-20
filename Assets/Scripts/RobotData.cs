using UnityEngine;

[CreateAssetMenu(fileName = "Robot Data", menuName = "Rapid Sim/Robot Data", order = 0)]
public class RobotData : ScriptableObject
{
    public NeuralNetwork[] Data;
}