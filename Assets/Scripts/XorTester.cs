using UnityEngine;

public class XorTester : MonoBehaviour
{
    private void Start()
    {
        const int steps = 5000;

        //const int trainingSteps = steps * 8;
        
        NeuralNetwork net = new(new[] { 3, 25, 25, 1 });

        for (int i = 0; i < steps; i++)
        {
            net.FeedForward(new float[] { 0, 0, 0 });
            net.BackProp(new float[] { 0 });
            
            net.FeedForward(new float[] { 0, 0, 1 });
            net.BackProp(new float[] { 1 });
            
            net.FeedForward(new float[] { 0, 1, 0 });
            net.BackProp(new float[] { 1 });
            
            net.FeedForward(new float[] { 0, 1, 1 });
            net.BackProp(new float[] { 0 });
            
            net.FeedForward(new float[] { 1, 0, 0 });
            net.BackProp(new float[] { 1 });
            
            net.FeedForward(new float[] { 1, 0, 1 });
            net.BackProp(new float[] { 0 });
            
            net.FeedForward(new float[] { 1, 1, 0 });
            net.BackProp(new float[] { 0 });
            
            net.FeedForward(new float[] { 1, 1, 1 });
            net.BackProp(new float[] { 1 });
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