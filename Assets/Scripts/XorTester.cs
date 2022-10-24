using UnityEngine;

public class XorTester : MonoBehaviour
{
    [SerializeField]
    private NeuralNetworkData data;
    
    private void Start()
    {
        int trainingStep = 1;

        NeuralNetwork net;

        if (data != null && !string.IsNullOrWhiteSpace(data.json))
        {
            net = JsonUtility.FromJson<NeuralNetwork>(data.json);
        }
        else
        {
            net = new(new[] { 3, 25, 25, 1 });
            for (int i = 0; i < 100; i++)
            {
                net.FeedForward(new float[] { 0, 0, 0 });
                net.BackProp(new float[] { 0 }, trainingStep++);
            
                net.FeedForward(new float[] { 0, 0, 1 });
                net.BackProp(new float[] { 1 }, trainingStep++);
            
                net.FeedForward(new float[] { 0, 1, 0 });
                net.BackProp(new float[] { 1 }, trainingStep++);
            
                net.FeedForward(new float[] { 0, 1, 1 });
                net.BackProp(new float[] { 0 }, trainingStep++);
            
                net.FeedForward(new float[] { 1, 0, 0 });
                net.BackProp(new float[] { 1 }, trainingStep++);
            
                net.FeedForward(new float[] { 1, 0, 1 });
                net.BackProp(new float[] { 0 }, trainingStep++);
            
                net.FeedForward(new float[] { 1, 1, 0 });
                net.BackProp(new float[] { 0 }, trainingStep++);
            
                net.FeedForward(new float[] { 1, 1, 1 });
                net.BackProp(new float[] { 1 }, trainingStep++);
            }
            
            if (data != null)
            {
                string json = JsonUtility.ToJson(net, true);
                data.json = json;
            }
        }

        Debug.Log($"Expected: 0 | Predicted: {net.FeedForward(new float[] { 0, 0, 0 })[0]}");
        Debug.Log($"Expected: 1 | Predicted: {net.FeedForward(new float[] { 0, 0, 1 })[0]}");
        Debug.Log($"Expected: 1 | Predicted: {net.FeedForward(new float[] { 0, 1, 0 })[0]}");
        Debug.Log($"Expected: 0 | Predicted: {net.FeedForward(new float[] { 0, 1, 1 })[0]}");
        Debug.Log($"Expected: 1 | Predicted: {net.FeedForward(new float[] { 1, 0, 0 })[0]}");
        Debug.Log($"Expected: 0 | Predicted: {net.FeedForward(new float[] { 1, 0, 1 })[0]}");
        Debug.Log($"Expected: 0 | Predicted: {net.FeedForward(new float[] { 1, 1, 0 })[0]}");
        Debug.Log($"Expected: 1 | Predicted: {net.FeedForward(new float[] { 1, 1, 1 })[0]}");

        Destroy(gameObject);
    }
}