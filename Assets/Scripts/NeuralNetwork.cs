using System;
using System.Collections.Generic;
using System.Linq;
using Random = Unity.Mathematics.Random;

public class NeuralNetwork : IComparable<NeuralNetwork>
{
    public float Fitness;
    
    private readonly int[] _layers;
    private readonly float[][] _neurons;
    private readonly float[][] _bias;
    private readonly float[][][] _weights;

    private Random _random;

    public NeuralNetwork(int[] layers)
    {
        uint random = (uint) DateTime.Now.Millisecond;
        if (random == 0)
        {
            random = 1;
        }

        _random = new(random);
        
        _layers = new int[layers.Length];
        for (int i = 0; i < layers.Length; i++)
        {
            _layers[i] = layers[i];
        }

        _neurons = _layers.Select(layer => new float[layer]).ToArray();
        
        List<float[]> bias = new();
        for (int i = 0; i < _layers.Length; i++)
        {
            float[] layerBias = new float[_neurons[i].Length];
            for (int j = 0; j < _neurons[i].Length; j++)
            {
                layerBias[j] = _random.NextFloat(-1f, 1f);
            }
            bias.Add(layerBias);
        }
        _bias = bias.ToArray();

        List<float[][]> weightsList = new();
        for (int i = 1; i < _layers.Length; i++)
        {
            List<float[]> layerWeightList = new();
            int neuronsInPreviousLayer = _layers[i - 1];
            for (int j = 0; j < _neurons[i].Length; j++)
            {
                float[] neuronWeights = new float[neuronsInPreviousLayer];
                for (int k = 0; k < neuronsInPreviousLayer; k++)
                {
                    neuronWeights[k] = _random.NextFloat(-1, 1);
                }
                layerWeightList.Add(neuronWeights);
            }
            weightsList.Add(layerWeightList.ToArray());
        }
        _weights = weightsList.ToArray();
    }

    public float[] Activate(float[] inputs)
    {
        for (int i = 0; i < inputs.Length; i++)
        {
            _neurons[0][i] = inputs[i];
        }

        for (int i = 1; i < _layers.Length; i++)
        {
            for (int j = 0; j < _neurons[i].Length; j++)
            {
                float value = _bias[i][j];
                for (int k = 0; k < _neurons[i - 1].Length; k++)
                {
                    value += _weights[i - 1][j][k] * _neurons[i - 1][k];
                    
                }

                _neurons[i][j] = Tanh(value);
            }
        }

        return _neurons[^1];
    }

    public float[] LastOutput()
    {
        return _neurons[^1];
    }

    public void Randomize()
    {
        for (int i = 0; i < _weights.Length; i++)
        {
            for (int j = 0; j < _weights[i].Length; j++)
            {
                for (int k = 0; k < _weights[i][j].Length; k++)
                {
                    _weights[i][j][k] = _random.NextFloat(-1, 1);
                }
            }
        }

        for (int i = 0; i < _bias.Length; i++)
        {
            for (int j = 0; j < _bias[i].Length; j++)
            {
                _bias[i][j] = _random.NextFloat(-1, 1);
            }
        }
    }

    public void Crossover(NeuralNetwork parent1, NeuralNetwork parent2, float mutationChance)
    {
        for (int i = 0; i < _weights.Length; i++)
        {
            for (int j = 0; j < _weights[i].Length; j++)
            {
                for (int k = 0; k < _weights[i][j].Length; k++)
                {
                    if (_random.NextFloat(-1, 1) < mutationChance)
                    {
                        _weights[i][j][k] = _random.NextFloat(-1, 1);
                    }
                    else
                    {
                        _weights[i][j][k] = _random.NextBool() ? parent1._weights[i][j][k] : parent2._weights[i][j][k];
                    }
                }
            }
        }

        for (int i = 0; i < _bias.Length; i++)
        {
            for (int j = 0; j < _bias[i].Length; j++)
            {
                if (_random.NextFloat(-1, 1) < mutationChance)
                {
                    _bias[i][j] = _random.NextFloat(-1, 1);
                }
                else
                {
                    _bias[i][j] = _random.NextBool() ? parent1._bias[i][j] : parent2._bias[i][j];
                }
            }
        }
    }
    
    private static float Tanh(float value)
    {
        return (float) Math.Tanh(value);
    }
    
    public int CompareTo(NeuralNetwork other)
    {
        return Fitness > other.Fitness ? 1 : Fitness < other.Fitness ? -1 : 0;
    }
}