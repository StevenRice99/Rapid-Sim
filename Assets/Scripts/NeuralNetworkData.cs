using UnityEngine;

[CreateAssetMenu(fileName = "Neural Network", menuName = "Neural Network", order = 0)]
public class NeuralNetworkData : ScriptableObject
{
    [TextArea(15,20)]
    public string json;
}