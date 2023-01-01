using UnityEngine;

namespace RapidSim.Networks.Testers
{
    public class XorTester : MonoBehaviour
    {
        [SerializeField]
        [Tooltip("The neural network which should have an input of three and a final output of one.")]
        private NeuralNetwork network;
        
        private void Start()
        {
            if (!NeuralNetwork.Validate(this, network, 3, 1))
            {
                return;
            }

            bool result = true;
            while (result)
            {
                network.Train(new double[] { 0, 0, 0 }, new double[] { 0 });
                network.Train(new double[] { 0, 0, 1 }, new double[] { 1 });
                network.Train(new double[] { 0, 1, 0 }, new double[] { 1 });
                network.Train(new double[] { 0, 1, 1 }, new double[] { 0 });
                network.Train(new double[] { 1, 0, 0 }, new double[] { 1 });
                network.Train(new double[] { 1, 0, 1 }, new double[] { 0 });
                network.Train(new double[] { 1, 1, 0 }, new double[] { 0 });
                result = network.Train(new double[] { 1, 1, 1 }, new double[] { 1 });
            }

            Debug.Log($"Expected: 0 | Predicted: {network.Forward(new double[] { 0, 0, 0 })[0]}");
            Debug.Log($"Expected: 1 | Predicted: {network.Forward(new double[] { 0, 0, 1 })[0]}");
            Debug.Log($"Expected: 1 | Predicted: {network.Forward(new double[] { 0, 1, 0 })[0]}");
            Debug.Log($"Expected: 0 | Predicted: {network.Forward(new double[] { 0, 1, 1 })[0]}");
            Debug.Log($"Expected: 1 | Predicted: {network.Forward(new double[] { 1, 0, 0 })[0]}");
            Debug.Log($"Expected: 0 | Predicted: {network.Forward(new double[] { 1, 0, 1 })[0]}");
            Debug.Log($"Expected: 0 | Predicted: {network.Forward(new double[] { 1, 1, 0 })[0]}");
            Debug.Log($"Expected: 1 | Predicted: {network.Forward(new double[] { 1, 1, 1 })[0]}");

            double[][] inputs =
            {
                new double[]
                {
                    0, 0, 0
                },
                new double[]
                {
                    0, 0, 1
                },
                new double[]
                {
                    0, 1, 0
                },
                new double[]
                {
                    0, 1, 1
                },
                new double[]
                {
                    1, 0, 0
                },
                new double[]
                {
                    1, 0, 1
                },
                new double[]
                {
                    1, 1, 0
                },
                new double[]
                {
                    1, 1, 1
                }
            };

            double[][] outputs =
            {
                new double[]
                {
                    0
                },
                new double[]
                {
                    1
                },
                new double[]
                {
                    1
                },
                new double[]
                {
                    0
                },
                new double[]
                {
                    1
                },
                new double[]
                {
                    0
                },
                new double[]
                {
                    0
                },
                new double[]
                {
                    1
                }
            };

            Debug.Log($"Accuracy: {network.Test(inputs, outputs)}");

            Destroy(gameObject);
        }
    }
}