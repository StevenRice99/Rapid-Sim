using System;

namespace RapidSim.Networks
{
    [Serializable]
    public struct NeuralNetwork
    {
        public Layer[] layers;
        public float beta1;
        public float beta2;
        public float epsilon;
        public float eta;
        public int step;
    
        public NeuralNetwork(int[] layer, float eta = 0.01f, float beta1 = 0.9f, float beta2 = 0.999f, float epsilon = 0.00000001f)
        {
            layers = new Layer[layer.Length - 1];

            for (int i = 0; i < layers.Length; i++)
            {
                layers[i] = new(layer[i], layer[i + 1]);
            }
        
            this.beta1 = beta1;
            this.beta2 = beta2;
            this.epsilon = epsilon;
            this.eta = eta;
            step = 0;
        }

        public float[] Forward(float[] inputs)
        {
            layers[0].Forward(inputs);
            for (int i = 1; i < layers.Length; i++)
            {
                layers[i].Forward(layers[i - 1].outputs);
            }

            return layers[^1].outputs;
        }

        public void Train(float[] inputs, float[] expected)
        {
            Forward(inputs);
            Backward(expected);
        }

        public float Test(float[] inputs, float[] expected)
        {
            float[] results = Forward(inputs);
            float accuracy = 0;
            for (int i = 0; i < expected.Length; i++)
            {
                accuracy += 1 - (expected[i] - results[i]);
            }

            return accuracy / expected.Length;
        }

        public float Test(float[][] inputs, float[][] expected)
        {
            float accuracy = 0;
            for (int i = 0; i < inputs.Length; i++)
            {
                accuracy += Test(inputs[i], expected[i]);
            }

            return accuracy / expected.Length;
        }

        private void Backward(float[] expected)
        {
            layers[^1].BackwardOutput(expected);
            for (int i = layers.Length - 2; i >= 0; i--)
            {
                layers[i].BackwardHidden(layers[i + 1].deltaBias, layers[i + 1].weights);
            }

            step++;
            for (int i = 0; i < layers.Length; i++)
            {
                layers[i].Optimize(step, eta, beta1, beta2, epsilon);
            }
        }

        public void Reset(float newEta = 0.01f, float newBeta1 = 0.9f, float newBeta2 = 0.999f, float newEpsilon = 0.00000001f)
        {
            for (int i = 0; i < layers.Length; i++)
            {
                layers[i].Reset();
            }
        
            beta1 = newBeta1;
            beta2 = newBeta2;
            epsilon = newEpsilon;
            eta = newEta;
            step = 0;
        }

        public void SetOptimization(float newEta = 0.01f, float newBeta1 = 0.9f, float newBeta2 = 0.999f, float newEpsilon = 0.00000001f)
        {
            for (int i = 0; i < layers.Length; i++)
            {
                layers[i].ResetOptimization();
            }
        
            beta1 = newBeta1;
            beta2 = newBeta2;
            epsilon = newEpsilon;
            eta = newEta;
            step = 0;
        }

        public override string ToString()
        {
            int neurons = 0;
            int parameters = 0;

            for (int i = 0; i < layers.Length; i++)
            {
                neurons += layers[i].outputs.Length;
                parameters += layers[i].NumberOfParameters;
            }

            string s = $"Neural Network - Layers: {layers.Length} | Inputs: {layers[0].inputs.Length} | Outputs: {layers[^1].outputs.Length} | Neurons: {neurons} | Parameters: {parameters}";
            for (int i = 0; i < layers.Length; i++)
            {
                s += $"\nLayer {i + 1} - {layers[i]}";
            }
        
            return s;
        }
    }
}