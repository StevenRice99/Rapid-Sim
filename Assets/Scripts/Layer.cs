using Unity.Mathematics;

public readonly struct Layer
{
    public readonly float[] Outputs;
    public readonly float[,] Weights;
    public readonly float[] DeltaBias;

    private readonly float[] _inputs;
    private readonly int _numberOfInputs;
    private readonly int _numberOfOutputs;
    private readonly float[] _bias;
    private readonly float[,] _deltaWeights;
    private readonly float[,] _momentumDeltaWeights;
    private readonly float[,] _velocityDeltaWeights;
    private readonly float[] _momentumDeltaBias;
    private readonly float[] _velocityDeltaBias;

    public Layer(int numberOfInputs, int numberOfOutputs)
    {
        _numberOfInputs = numberOfInputs;
        _numberOfOutputs = numberOfOutputs;

        _inputs = new float[_numberOfInputs];
        Outputs = new float[_numberOfOutputs];

        Weights = new float[_numberOfOutputs, _numberOfInputs];
        _deltaWeights = new float[_numberOfOutputs, _numberOfInputs];
        _momentumDeltaWeights = new float[_numberOfOutputs, _numberOfInputs];
        _velocityDeltaWeights = new float[_numberOfOutputs, _numberOfInputs];

        _bias = new float[_numberOfOutputs];
        DeltaBias = new float[_numberOfOutputs];
        _momentumDeltaBias = new float[_numberOfOutputs];
        _velocityDeltaBias = new float[_numberOfOutputs];

        uint seed = (uint) System.DateTime.UtcNow.Ticks;
        if (seed == 0)
        {
            seed = 1;
        }

        Random random = new(seed);

        for (int i = 0; i < _numberOfOutputs; i++)
        {
            for (int j = 0; j < _numberOfInputs; j++)
            {
                Weights[i, j] = random.NextFloat(-0.5f, 0.5f);
                _momentumDeltaWeights[i, j] = 0;
                _velocityDeltaWeights[i, j] = 0;
            }
            
            _bias[i] = random.NextFloat(-0.5f, 0.5f);
            _momentumDeltaBias[i] = 0;
            _velocityDeltaBias[i] = 0;
        }
    }

    public float[] FeedForward(float[] input)
    {
        for (int i = 0; i < _inputs.Length; i++)
        {
            _inputs[i] = input[i];
        }

        for (int i = 0; i < _numberOfOutputs; i++)
        {
            Outputs[i] = _bias[i];
            for (int j = 0; j < _numberOfInputs; j++)
            {
                Outputs[i] += _inputs[j] * Weights[i, j];
            }

            Outputs[i] = Activation(Outputs[i]);
        }

        return Outputs;
    }

    public void BackPropOutput(float[] expected)
    {
        for (int i = 0; i < _numberOfOutputs; i++)
        {
            DeltaBias[i] = (Outputs[i] - expected[i]) * ActivationDerivative(Outputs[i]);
            
            for (int j = 0; j < _numberOfInputs; j++)
            {
                _deltaWeights[i, j] = DeltaBias[i] * _inputs[j];
            }
        }
    }

    public void BackPropHidden(float[] gammaForward, float[,] weightsForward)
    {
        for (int i = 0; i < _numberOfOutputs; i++)
        {
            DeltaBias[i] = 0;

            for (int j = 0; j < gammaForward.Length; j++)
            {
                DeltaBias[i] += gammaForward[j] * weightsForward[j, i];
            }

            DeltaBias[i] *= ActivationDerivative(Outputs[i]);
            
            for (int j = 0; j < _numberOfInputs; j++)
            {
                _deltaWeights[i, j] = DeltaBias[i] * _inputs[j];
            }
        }
    }

    public void Optimize(int t, float eta, float beta1, float beta2, float epsilon)
    {
        for (int i = 0; i < _numberOfOutputs; i++)
        {
            for (int j = 0; j < _numberOfInputs; j++)
            {
                _momentumDeltaWeights[i, j] = beta1 * _momentumDeltaWeights[i, j] + (1 - beta1) * _deltaWeights[i, j];
                _velocityDeltaWeights[i, j] = beta2 * _velocityDeltaWeights[i, j] + (1 - beta2) * math.pow(_deltaWeights[i, j], 2);
            
                float momentumDeltaWeightCorrection = _momentumDeltaWeights[i, j] / (1 - math.pow(beta1, t));
                float velocityDeltaWeightCorrection = _velocityDeltaWeights[i, j] / (1 - math.pow(beta2, t));
                
                Weights[i, j] -= eta * (momentumDeltaWeightCorrection / (math.sqrt(velocityDeltaWeightCorrection) + epsilon));
            }
            
            _momentumDeltaBias[i] = beta1 * _momentumDeltaBias[i] + (1 - beta1) * DeltaBias[i];
            _velocityDeltaBias[i] = beta2 * _velocityDeltaBias[i] + (1 - beta2) * DeltaBias[i];
            
            float momentumDeltaBiaCorrection = _momentumDeltaBias[i] / (1 - math.pow(beta1, t));
            float velocityDeltaBiaCorrection = _velocityDeltaBias[i] / (1 - math.pow(beta2, t));

            DeltaBias[i] -= eta * (momentumDeltaBiaCorrection / (math.sqrt(velocityDeltaBiaCorrection) + epsilon));
        }
    }

    private static float Activation(float value)
    {
        return math.tanh(value);
    }

    private static float ActivationDerivative(float value)
    {
        return 1 - value * value;
    }
}