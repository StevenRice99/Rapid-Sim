using UnityEngine;

namespace RapidSim.Networks
{
    [CreateAssetMenu(fileName = "Neural Network", menuName = "Neural Network", order = 0)]
    public class NeuralNetwork : ScriptableObject
    {
        public int[] layers = {1, 1};
        
        [Header("ADAM Optimizer")]
        [Range(0, 1)]
        public double learningRate = 0.001;
        [Range(0, 1)]
        public double beta1 = 0.9;
        [Range(0, 1)]
        public double beta2 = 0.999;
        [Range(0, 1)]
        public double epsilon = 1e-08;
        
        [Header("Training")]
        [Min(1)]
        public int maxSteps = 100;
        [Min(0)]
        public int step;
        
        private bool Setup => _layers is not {Length: 0};
        
        private Layer[] _layers;

        private void OnEnable()
        {
            OnValidate();
        }

        private void OnValidate()
        {
            if (step > maxSteps)
            {
                step = maxSteps;
            }
            
            bool create = _layers == null || _layers.Length != layers.Length - 1;

            if (!create)
            {
                for (int i = 0; i < _layers.Length; i++)
                {
                    if (_layers[i].numberOfInputs == layers[i] && _layers[i].numberOfOutputs == layers[i + 1])
                    {
                        continue;
                    }

                    create = true;
                    break;
                }
            }
            
            if (create)
            {
                Create();
            }
        }

        public void Create()
        {
            _layers = new Layer[layers.Length - 1];

            for (int i = 0; i < _layers.Length; i++)
            {
                _layers[i] = new(layers[i], layers[i + 1]);
            }

            step = 0;
            
            Debug.Log($"Initialized {BuildString(name)}");
        }

        public double[] Forward(double[] inputs)
        {
            _layers[0].Forward(inputs);
            for (int i = 1; i < _layers.Length; i++)
            {
                _layers[i].Forward(_layers[i - 1].outputs);
            }

            return _layers[^1].outputs;
        }

        public bool Train(double[] inputs, double[] expected)
        {
            if (step >= maxSteps)
            {
                return false;
            }
            
            Forward(inputs);
            Backward(expected);

            return true;
        }

        public double Test(double[] inputs, double[] expected)
        {
            double[] results = Forward(inputs);
            double accuracy = 0;
            for (int i = 0; i < expected.Length; i++)
            {
                accuracy += 1 - (expected[i] - results[i]);
            }

            return accuracy / expected.Length;
        }

        public double Test(double[][] inputs, double[][] expected)
        {
            double accuracy = 0;
            for (int i = 0; i < inputs.Length; i++)
            {
                accuracy += Test(inputs[i], expected[i]);
            }

            return accuracy / expected.Length;
        }

        private void Backward(double[] expected)
        {
            _layers[^1].BackwardOutput(expected);
            for (int i = _layers.Length - 2; i >= 0; i--)
            {
                _layers[i].BackwardHidden(_layers[i + 1].deltaBias, _layers[i + 1].weights);
            }

            step++;
            for (int i = 0; i < _layers.Length; i++)
            {
                _layers[i].Optimize(step, learningRate, beta1, beta2, epsilon);
            }
        }

        public override string ToString()
        {
            return BuildString(name);
        }

        private string BuildString(string title)
        {
            if (!Setup)
            {
                return "Network not setup.";
            }
            
            int neurons = 0;
            int parameters = 0;
            
            for (int i = 0; i < _layers.Length; i++)
            {
                neurons += _layers[i].outputs.Length;
                parameters += _layers[i].NumberOfParameters;
            }

            string s = $"{title} - Layers: {_layers.Length} | Inputs: {_layers[0].inputs.Length} | Outputs: {_layers[^1].outputs.Length} | Neurons: {neurons} | Parameters: {parameters}";
            for (int i = 0; i < _layers.Length; i++)
            {
                s += $"\nLayer {i + 1} - {_layers[i]}";
            }
        
            return s;
        }
    }
}