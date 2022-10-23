﻿using Unity.Mathematics;

public struct Layer
{
    private const float LearningRate = 0.00333f;

    public readonly float[] Outputs;
    public readonly float[,] Weights;
    public readonly float[] Gamma;
    
    private readonly int _numberOfInputs;
    private readonly int _numberOfOutputs;
    private readonly float[,] _weightsDelta;
    private readonly float[] _error;
    
    private float[] _inputs;

    public Layer(int numberOfInputs, int numberOfOutputs)
    {
        _numberOfInputs = numberOfInputs;
        _numberOfOutputs = numberOfOutputs;

        _inputs = new float[numberOfInputs];
        Outputs = new float[numberOfOutputs];

        Weights = new float[_numberOfOutputs, _numberOfInputs];
        _weightsDelta = new float[_numberOfOutputs, _numberOfInputs];
        
        Gamma = new float[numberOfOutputs];
        _error = new float[numberOfOutputs];

        Random random = new(math.max((uint) System.DateTime.UtcNow.Ticks, 1));

        for (int i = 0; i < _numberOfOutputs; i++)
        {
            for (int j = 0; j < _numberOfInputs; j++)
            {
                Weights[i, j] = random.NextFloat(-0.5f, 0.5f);
            }
        }
    }

    public float[] FeedForward(float[] input)
    {
        _inputs = input;

        for (int i = 0; i < _numberOfOutputs; i++)
        {
            Outputs[i] = 0;
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
            _error[i] = Outputs[i] - expected[i];
        }

        for (int i = 0; i < _numberOfOutputs; i++)
        {
            Gamma[i] = _error[i] * ActivationDerivative(Outputs[i]);
        }

        for (int i = 0; i < _numberOfOutputs; i++)
        {
            for (int j = 0; j < _numberOfInputs; j++)
            {
                _weightsDelta[i, j] = Gamma[i] * _inputs[j];
            }
        }
    }

    public void BackPropHidden(float[] gammaForward, float[,] weightsForward)
    {
        for (int i = 0; i < _numberOfOutputs; i++)
        {
            Gamma[i] = 0;

            for (int j = 0; j < gammaForward.Length; j++)
            {
                Gamma[i] += gammaForward[j] * weightsForward[j, i];
            }

            Gamma[i] *= ActivationDerivative(Outputs[i]);
        }
        
        for (int i = 0; i < _numberOfOutputs; i++)
        {
            for (int j = 0; j < _numberOfInputs; j++)
            {
                _weightsDelta[i, j] = Gamma[i] * _inputs[j];
            }
        }
    }

    public void UpdateWeights()
    {
        for (int i = 0; i < _numberOfOutputs; i++)
        {
            for (int j = 0; j < _numberOfInputs; j++)
            {
                Weights[i, j] -= _weightsDelta[i, j] * LearningRate;
            }
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