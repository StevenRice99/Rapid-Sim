using System;

namespace RapidSim.Networks
{
    [Serializable]
    public struct NeuralNetwork
    {
        public Layer[] layers;
    
        public NeuralNetwork(int[] layer)
        {
            layers = new Layer[layer.Length - 1];

            for (int i = 0; i < layers.Length; i++)
            {
                layers[i] = new(layer[i], layer[i + 1]);
            }
        }

        public double[] Forward(double[] values)
        {
            for (int i = 0; i < layers.Length; i++)
            {
                values = layers[i].Forward(values);
            }

            return values;
        }

        public override string ToString()
        {
            string s = $"Neural Network - Layers: {layers.Length}";
            for (int i = 0; i < layers.Length; i++)
            {
                s += $"\nLayer {i + 1} - {layers[i]}";
            }
        
            return s;
        }
    }
}