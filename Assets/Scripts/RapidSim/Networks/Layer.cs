using System;

namespace RapidSim.Networks
{
    [Serializable]
    public struct Layer
    {
        public Neuron[] neurons;

        public Layer(int numberOfInputs, int numberOfOutputs)
        {
            neurons = new Neuron[numberOfOutputs];
            for (int i = 0; i < numberOfOutputs; i++)
            {
                neurons[i] = new(numberOfInputs);
            }
        }

        public double[] Forward(double[] values)
        {
            double[] outputs = new double[values.Length];
            for (int i = 0; i < neurons.Length; i++)
            {
                outputs[i] = neurons[i].Forward(values);
            }

            return outputs;
        }

        public override string ToString()
        {
            return $"Inputs: {neurons[0].weights.Length} | Outputs: {neurons.Length}";
        }
    }
}