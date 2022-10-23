public readonly struct NeuralNetwork
{
    private readonly Layer[] _layers;
    private readonly float _beta1;
    private readonly float _beta2;
    private readonly float _epsilon;
    private readonly float _eta;
    
    public NeuralNetwork(int[] layer, float eta = 0.01f, float beta1 = 0.9f, float beta2 = 0.999f, float epsilon = 0.00000001f)
    {
        _layers = new Layer[layer.Length - 1];

        for (int i = 0; i < _layers.Length; i++)
        {
            _layers[i] = new(layer[i], layer[i + 1]);
        }
        
        _beta1 = beta1;
        _beta2 = beta2;
        _epsilon = epsilon;
        _eta = eta;
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

    public void BackProp(float[] expected, int t)
    {
        _layers[^1].BackPropOutput(expected);
        for (int i = _layers.Length - 2; i >= 0; i--)
        {
            _layers[i].BackPropHidden(_layers[i + 1].DeltaBias, _layers[i + 1].Weights);
        }

        for (int i = 0; i < _layers.Length; i++)
        {
            _layers[i].Optimize(t, _eta, _beta1, _beta2, _epsilon);
        }
    }
}