using System;

namespace RapidSim.Networks
{
    [Serializable]
    public struct Neuron
    {
        public double[] weights;
        public double bias;

        public Neuron(int numberOfInputs)
        {
            weights = new double[numberOfInputs];
            for (int i = 0; i < numberOfInputs; i++)
            {
                weights[i] = 0;
            }
            bias = 0;
        }

        public double Forward(double[] values)
        {
            double output = bias;
            for (int i = 0; i < weights.Length; i++)
            {
                output += values[i] * weights[i];
            }

            return output > 0 ? output : 0;
        }
    }
}