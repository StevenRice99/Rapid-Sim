using System;

[Serializable]
public struct NeuralNetwork
{
    public Layer[] layers;
    public float beta1;
    public float beta2;
    public float epsilon;
    public float eta;
    
    public NeuralNetwork(int[] layer, float eta = 0.01f, float beta1 = 0.9f, float beta2 = 0.999f, float epsilon = 0.00000001f)
    {
        layers = new Layer[layer.Length - 1];

        for (int i = 0; i < layers.Length; i++)
        {
            layers[i] = new(layer[i], layer[i + 1]);
        }
        
        this.beta1 = beta1;
        this.beta2 = beta2;
        this.epsilon = epsilon;
        this.eta = eta;
    }

    public float[] FeedForward(float[] inputs)
    {
        layers[0].FeedForward(inputs);
        for (int i = 1; i < layers.Length; i++)
        {
            layers[i].FeedForward(layers[i - 1].outputs);
        }

        return layers[^1].outputs;
    }

    public void BackProp(float[] expected, int t)
    {
        layers[^1].BackPropOutput(expected);
        for (int i = layers.Length - 2; i >= 0; i--)
        {
            layers[i].BackPropHidden(layers[i + 1].deltaBias, layers[i + 1].weights);
        }

        for (int i = 0; i < layers.Length; i++)
        {
            layers[i].Optimize(t, eta, beta1, beta2, epsilon);
        }
    }
}