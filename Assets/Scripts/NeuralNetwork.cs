public struct NeuralNetwork
{
    private readonly Layer[] _layers;
    
    public NeuralNetwork(int[] layer)
    {
        _layers = new Layer[layer.Length - 1];

        for (int i = 0; i < _layers.Length; i++)
        {
            _layers[i] = new(layer[i], layer[i + 1]);
        }
    }

    public float[] FeedForward(float[] inputs)
    {
        _layers[0].FeedForward(inputs);
        for (int i = 1; i < _layers.Length; i++)
        {
            _layers[i].FeedForward(_layers[i - 1].Outputs);
        }

        return _layers[^1].Outputs;
    }

    public void BackProp(float[] expected)
    {
        _layers[^1].BackPropOutput(expected);
        for (int i = _layers.Length - 2; i >= 0; i--)
        {
            _layers[i].BackPropHidden(_layers[i + 1].OutputGradient, _layers[i + 1].Weights);
        }

        for (int i = 0; i < _layers.Length; i++)
        {
            _layers[i].UpdateWeights();
        }
    }
}