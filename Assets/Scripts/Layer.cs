using Unity.Mathematics;

public struct Layer
{
    private const float LearningRate = 0.00333f;

    public readonly float[] Outputs;
    public readonly float[,] Weights;
    public readonly float[] OutputGradient;
    
    private readonly int _numberOfInputs;
    private readonly int _numberOfOutputs;
    private readonly float[] _bias;
    private readonly float[,] _weightsGradient;

    private float[] _inputs;

    public Layer(int numberOfInputs, int numberOfOutputs)
    {
        _numberOfInputs = numberOfInputs;
        _numberOfOutputs = numberOfOutputs;

        _inputs = new float[numberOfInputs];
        Outputs = new float[numberOfOutputs];

        Weights = new float[_numberOfOutputs, _numberOfInputs];
        _weightsGradient = new float[_numberOfOutputs, _numberOfInputs];

        _bias = new float[_numberOfOutputs];
        
        OutputGradient = new float[numberOfOutputs];

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
            }
            
            _bias[i] = random.NextFloat(-0.5f, 0.5f);
        }
    }

    public float[] FeedForward(float[] input)
    {
        _inputs = input;

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
            // ERROR
            OutputGradient[i] = (Outputs[i] - expected[i]) * ActivationDerivative(Outputs[i]);
            
            for (int j = 0; j < _numberOfInputs; j++)
            {
                _weightsGradient[i, j] = OutputGradient[i] * _inputs[j];
            }
        }
    }

    public void BackPropHidden(float[] gammaForward, float[,] weightsForward)
    {
        for (int i = 0; i < _numberOfOutputs; i++)
        {
            OutputGradient[i] = 0;

            for (int j = 0; j < gammaForward.Length; j++)
            {
                OutputGradient[i] += gammaForward[j] * weightsForward[j, i];
            }

            OutputGradient[i] *= ActivationDerivative(Outputs[i]);
            
            for (int j = 0; j < _numberOfInputs; j++)
            {
                _weightsGradient[i, j] = OutputGradient[i] * _inputs[j];
            }
        }
    }

    public void UpdateWeights()
    {
        for (int i = 0; i < _numberOfOutputs; i++)
        {
            for (int j = 0; j < _numberOfInputs; j++)
            {
                Weights[i, j] -= _weightsGradient[i, j] * LearningRate;
            }

            _bias[i] -= OutputGradient[i] * LearningRate;
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