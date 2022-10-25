using UnityEngine;

namespace RapidSim.Networks
{
    [CreateAssetMenu(fileName = "Neural Network", menuName = "Neural Network", order = 0)]
    public class NeuralNetworkData : ScriptableObject
    {
        [TextArea(15,20)]
        public string json;

        public bool HasModel => !string.IsNullOrWhiteSpace(json);

        public void Save(NeuralNetwork net)
        {
            json = JsonUtility.ToJson(net, true);
        }

        public NeuralNetwork Load()
        {
            return JsonUtility.FromJson<NeuralNetwork>(json);
        }
    }
}