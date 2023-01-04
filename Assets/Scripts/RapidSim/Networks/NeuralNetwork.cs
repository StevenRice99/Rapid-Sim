using Unity.Mathematics;
using UnityEditor;
using UnityEngine;

namespace RapidSim.Networks
{
    [CreateAssetMenu(fileName = "Neural Network", menuName = "Neural Network", order = 0)]
    public class NeuralNetwork : ScriptableObject
    {
        [Tooltip("The architecture of the neural network.")]
        public int[] architecture = {1, 1};
        
        [Header("ADAM Optimizer")]
        [Tooltip("The learning rate for the optimizer.")]
        [Range(0, 1)]
        public double learningRate = 0.001;
        
        [Tooltip("The first coefficient used for computing running averages of the gradient and its square.")]
        [Range(0, 1)]
        public double beta1 = 0.9;
        
        [Tooltip("The second coefficient used for computing running averages of the gradient and its square.")]
        [Range(0, 1)]
        public double beta2 = 0.999;
        
        [Tooltip("Term added to the denominator to improve numerical stability.")]
        [Range(0, 1)]
        public double epsilon = 1e-08;
        
        [Header("Training")]
        [Tooltip("The maximum number of epochs to perform training for.")]
        [Min(1)]
        public int epochs = 100;
        
        [Tooltip("The current training epoch.")]
        [Min(0)]
        public int currentEpoch;
        
        private bool Setup => layers is not {Length: 0};
        
        [SerializeField]
        [HideInInspector]
        private Layer[] layers;

        private void OnValidate()
        {
            if (currentEpoch > epochs)
            {
                currentEpoch = epochs;
            }
            
            bool create = layers == null || layers.Length != architecture.Length - 1;

            if (!create)
            {
                for (int i = 0; i < layers.Length; i++)
                {
                    if (layers[i].numberOfInputs == architecture[i] && layers[i].numberOfOutputs == architecture[i + 1])
                    {
                        continue;
                    }

                    create = true;
                    break;
                }
            }

            if (!create)
            {
                return;
            }

            layers = new Layer[architecture.Length - 1];

            for (int i = 0; i < layers.Length; i++)
            {
                layers[i] = new(architecture[i], architecture[i + 1]);
            }

            currentEpoch = 0;

            Debug.Log($"Initialized {BuildString(name)}");
        }

        public static bool Validate(Component attached, NeuralNetwork network, int requiredInputs, int requiredOutputs)
        {
            if (network == null)
            {
                Debug.LogError($"\"{attached.name}\" does not have a neural network attached to it.");
                return Stop(attached);
            }
            
            if (!network.Setup)
            {
                Debug.LogError($"Neural network \"{network.name}\" attached to \"{attached.name}\" is not setup.");
                return Stop(attached);
            }

            if (network.layers[0].numberOfInputs != requiredInputs)
            {
                Debug.LogError($"Neural network \"{network.name}\" attached to \"{attached.name}\" requires {requiredInputs} inputs but has {network.layers[0].numberOfInputs} inputs.");
                return Stop(attached);
            }

            if (network.layers[^1].numberOfOutputs != requiredOutputs)
            {
                Debug.LogError($"Neural network \"{network.name}\" attached to \"{attached.name}\" requires {requiredOutputs} outputs but has {network.layers[^1].numberOfOutputs} outputs.");
                return Stop(attached);
            }

            return true;
        }

        private static bool Stop(Component attached)
        {
#if UNITY_EDITOR
            EditorApplication.isPlaying = false;
#else
            Application.Quit();
#endif
            Destroy(attached.gameObject);
            return false;
        }

        public double[] Forward(double[] inputs)
        {
            layers[0].Forward(inputs);
            for (int i = 1; i < layers.Length; i++)
            {
                layers[i].Forward(layers[i - 1].outputs);
            }

            return layers[^1].outputs;
        }

        public bool Train(Dataset dataset)
        {
            if (currentEpoch >= epochs)
            {
                return false;
            }
            
            for (int i = 0; i < dataset.dataPoints.Count; i++)
            {
                Forward(dataset.dataPoints[i].inputs);
                Backward(dataset.dataPoints[i].outputs);
            }
            
            currentEpoch++;

            return true;
        }

        public double Test(Dataset dataset)
        {
            double totalAccuracy = 0;
            
            for (int i = 0; i < dataset.Size; i++)
            {
                double[] results = Forward(dataset.dataPoints[i].inputs);
                double accuracy = 0;
                for (int j = 0; j < dataset.dataPoints[i].outputs.Length; j++)
                {
                    Debug.Log($"Data Point {i + 1} | Joint {j + 1} | Expected = {dataset.dataPoints[i].outputs[j]} | Result = {results[j]}");
                    accuracy += math.abs(math.max(dataset.dataPoints[i].outputs[j], results[j]) - math.min(dataset.dataPoints[i].outputs[j], results[j]));
                }

                Debug.Log($"Data Point {i + 1} = {accuracy}");
                totalAccuracy += accuracy / dataset.dataPoints[i].outputs.Length;
            }

            return 1 - totalAccuracy / dataset.Size;
        }

        private void Backward(double[] expected)
        {
            layers[^1].BackwardOutput(expected);
            for (int i = layers.Length - 2; i >= 0; i--)
            {
                layers[i].BackwardHidden(layers[i + 1].deltaBias, layers[i + 1].weights);
            }

            for (int i = 0; i < layers.Length; i++)
            {
                layers[i].Optimize(currentEpoch, learningRate, beta1, beta2, epsilon);
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
                return $"{title} not setup.";
            }
            
            int neurons = 0;
            int parameters = 0;
            
            for (int i = 0; i < layers.Length; i++)
            {
                neurons += layers[i].outputs.Length;
                parameters += layers[i].NumberOfParameters;
            }

            string s = $"{title} - Layers: {layers.Length} | Inputs: {layers[0].inputs.Length} | Outputs: {layers[^1].outputs.Length} | Neurons: {neurons} | Parameters: {parameters}";
            for (int i = 0; i < layers.Length; i++)
            {
                s += $"\nLayer {i + 1} - {layers[i]}";
            }
        
            return s;
        }
    }
}