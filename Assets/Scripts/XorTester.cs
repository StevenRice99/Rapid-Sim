using UnityEngine;

public class XorTester : MonoBehaviour
{
    [SerializeField]
    private NeuralNetworkData data;
    
    private void Start()
    {
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
                net.Train(new float[] { 0, 0, 0 }, new float[] { 0 });
                net.Train(new float[] { 0, 0, 1 }, new float[] { 1 });
                net.Train(new float[] { 0, 1, 0 }, new float[] { 1 });
                net.Train(new float[] { 0, 1, 1 }, new float[] { 0 });
                net.Train(new float[] { 1, 0, 0 }, new float[] { 1 });
                net.Train(new float[] { 1, 0, 1 }, new float[] { 0 });
                net.Train(new float[] { 1, 1, 0 }, new float[] { 0 });
                net.Train(new float[] { 1, 1, 1 }, new float[] { 1 });
            }
            
            if (data != null)
            {
                string json = JsonUtility.ToJson(net, true);
                data.json = json;
            }
        }

        Debug.Log($"Expected: 0 | Predicted: {net.Forward(new float[] { 0, 0, 0 })[0]}");
        Debug.Log($"Expected: 1 | Predicted: {net.Forward(new float[] { 0, 0, 1 })[0]}");
        Debug.Log($"Expected: 1 | Predicted: {net.Forward(new float[] { 0, 1, 0 })[0]}");
        Debug.Log($"Expected: 0 | Predicted: {net.Forward(new float[] { 0, 1, 1 })[0]}");
        Debug.Log($"Expected: 1 | Predicted: {net.Forward(new float[] { 1, 0, 0 })[0]}");
        Debug.Log($"Expected: 0 | Predicted: {net.Forward(new float[] { 1, 0, 1 })[0]}");
        Debug.Log($"Expected: 0 | Predicted: {net.Forward(new float[] { 1, 1, 0 })[0]}");
        Debug.Log($"Expected: 1 | Predicted: {net.Forward(new float[] { 1, 1, 1 })[0]}");

        float[][] inputs =
        {
            new float[]
            {
                0, 0, 0
            },
            new float[]
            {
                0, 0, 1
            },
            new float[]
            {
                0, 1, 0
            },
            new float[]
            {
                0, 1, 1
            },
            new float[]
            {
                1, 0, 0
            },
            new float[]
            {
                1, 0, 1
            },
            new float[]
            {
                1, 1, 0
            },
            new float[]
            {
                1, 1, 1
            }
        };

        float[][] outputs =
        {
            new float[]
            {
                0
            },
            new float[]
            {
                1
            },
            new float[]
            {
                1
            },
            new float[]
            {
                0
            },
            new float[]
            {
                1
            },
            new float[]
            {
                0
            },
            new float[]
            {
                0
            },
            new float[]
            {
                1
            }
        };

        Debug.Log($"Accuracy: {net.Test(inputs, outputs)}");
        Debug.Log(net);

        Destroy(gameObject);
    }
}