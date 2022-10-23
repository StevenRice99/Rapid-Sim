using UnityEngine;

public class XorTester : MonoBehaviour
{
    private void Start()
    {
        int trainingStep = 1;
        
        NeuralNetwork net = new(new[] { 3, 25, 25, 1 });

        for (int i = 0; i < 10000; i++)
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