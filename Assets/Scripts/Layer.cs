﻿using System;
using Unity.Mathematics;
using Random = Unity.Mathematics.Random;

[Serializable]
public struct Layer
{
    public float[] outputs;
    public WrappedArray[] weights;
    public float[] deltaBias;

    public float[] inputs;
    public int numberOfInputs;
    public int numberOfOutputs;
    public float[] bias;
    public WrappedArray[] deltaWeights;
    public WrappedArray[] momentumDeltaWeights;
    public WrappedArray[] velocityDeltaWeights;
    public float[] momentumDeltaBias;
    public float[] velocityDeltaBias;

    public Layer(int numberOfInputs, int numberOfOutputs)
    {
        this.numberOfInputs = numberOfInputs;
        this.numberOfOutputs = numberOfOutputs;

        inputs = new float[this.numberOfInputs];
        outputs = new float[this.numberOfOutputs];

        weights = new WrappedArray[this.numberOfOutputs];
        deltaWeights = new WrappedArray[this.numberOfOutputs];
        momentumDeltaWeights = new WrappedArray[this.numberOfOutputs];
        velocityDeltaWeights = new WrappedArray[this.numberOfOutputs];
        for (int i = 0; i < this.numberOfOutputs; i++)
        {
            weights[i].data = new float[this.numberOfInputs];
            deltaWeights[i].data = new float[this.numberOfInputs];
            momentumDeltaWeights[i].data = new float[this.numberOfInputs];
            velocityDeltaWeights[i].data = new float[this.numberOfInputs];
        }

        bias = new float[this.numberOfOutputs];
        deltaBias = new float[this.numberOfOutputs];
        momentumDeltaBias = new float[this.numberOfOutputs];
        velocityDeltaBias = new float[this.numberOfOutputs];

        uint seed = (uint) DateTime.UtcNow.Ticks;
        if (seed == 0)
        {
            seed = 1;
        }

        Random random = new(seed);

        for (int i = 0; i < this.numberOfOutputs; i++)
        {
            for (int j = 0; j < this.numberOfInputs; j++)
            {
                weights[i].data[j] = random.NextFloat(-0.5f, 0.5f);
                momentumDeltaWeights[i].data[j] = 0;
                velocityDeltaWeights[i].data[j] = 0;
            }
            
            bias[i] = random.NextFloat(-0.5f, 0.5f);
            momentumDeltaBias[i] = 0;
            velocityDeltaBias[i] = 0;
        }
    }

    public float[] FeedForward(float[] input)
    {
        for (int i = 0; i < inputs.Length; i++)
        {
            inputs[i] = input[i];
        }

        for (int i = 0; i < numberOfOutputs; i++)
        {
            outputs[i] = bias[i];
            for (int j = 0; j < numberOfInputs; j++)
            {
                outputs[i] += inputs[j] * weights[i].data[j];
            }

            outputs[i] = Activation(outputs[i]);
        }

        return outputs;
    }

    public void BackPropOutput(float[] expected)
    {
        for (int i = 0; i < numberOfOutputs; i++)
        {
            deltaBias[i] = (outputs[i] - expected[i]) * ActivationDerivative(outputs[i]);
            
            for (int j = 0; j < numberOfInputs; j++)
            {
                deltaWeights[i].data[j] = deltaBias[i] * inputs[j];
            }
        }
    }

    public void BackPropHidden(float[] gammaForward, WrappedArray[] weightsForward)
    {
        for (int i = 0; i < numberOfOutputs; i++)
        {
            deltaBias[i] = 0;

            for (int j = 0; j < gammaForward.Length; j++)
            {
                deltaBias[i] += gammaForward[j] * weightsForward[j].data[i];
            }

            deltaBias[i] *= ActivationDerivative(outputs[i]);
            
            for (int j = 0; j < numberOfInputs; j++)
            {
                deltaWeights[i].data[j] = deltaBias[i] * inputs[j];
            }
        }
    }

    public void Optimize(int t, float eta, float beta1, float beta2, float epsilon)
    {
        for (int i = 0; i < numberOfOutputs; i++)
        {
            for (int j = 0; j < numberOfInputs; j++)
            {
                momentumDeltaWeights[i].data[j] = beta1 * momentumDeltaWeights[i].data[j] + (1 - beta1) * deltaWeights[i].data[j];
                velocityDeltaWeights[i].data[j] = beta2 * velocityDeltaWeights[i].data[j] + (1 - beta2) * math.pow(deltaWeights[i].data[j], 2);
            
                float momentumDeltaWeightCorrection = momentumDeltaWeights[i].data[j] / (1 - math.pow(beta1, t));
                float velocityDeltaWeightCorrection = velocityDeltaWeights[i].data[j] / (1 - math.pow(beta2, t));
                
                weights[i].data[j] -= eta * (momentumDeltaWeightCorrection / (math.sqrt(velocityDeltaWeightCorrection) + epsilon));
            }
            
            momentumDeltaBias[i] = beta1 * momentumDeltaBias[i] + (1 - beta1) * deltaBias[i];
            velocityDeltaBias[i] = beta2 * velocityDeltaBias[i] + (1 - beta2) * deltaBias[i];
            
            float momentumDeltaBiaCorrection = momentumDeltaBias[i] / (1 - math.pow(beta1, t));
            float velocityDeltaBiaCorrection = velocityDeltaBias[i] / (1 - math.pow(beta2, t));

            deltaBias[i] -= eta * (momentumDeltaBiaCorrection / (math.sqrt(velocityDeltaBiaCorrection) + epsilon));
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