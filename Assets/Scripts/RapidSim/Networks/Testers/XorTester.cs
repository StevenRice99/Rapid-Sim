using UnityEngine;

namespace RapidSim.Networks.Testers
{
    public class XorTester : MonoBehaviour
    {
        [SerializeField]
        private NeuralNetworkData data;
    
        private void Start()
        {
            NeuralNetwork net;

            if (data != null && data.HasModel)
            {
                net = data.Load();
            }
            else
            {
                net = new(new[] { 3, 25, 25, 1 });
                for (int i = 0; i < 100; i++)
                {
                    net.Train(new double[] { 0, 0, 0 }, new double[] { 0 });
                    net.Train(new double[] { 0, 0, 1 }, new double[] { 1 });
                    net.Train(new double[] { 0, 1, 0 }, new double[] { 1 });
                    net.Train(new double[] { 0, 1, 1 }, new double[] { 0 });
                    net.Train(new double[] { 1, 0, 0 }, new double[] { 1 });
                    net.Train(new double[] { 1, 0, 1 }, new double[] { 0 });
                    net.Train(new double[] { 1, 1, 0 }, new double[] { 0 });
                    net.Train(new double[] { 1, 1, 1 }, new double[] { 1 });
                }
            
                if (data != null)
                {
                    data.Save(net);
                }
            }

            Debug.Log($"Expected: 0 | Predicted: {net.Forward(new double[] { 0, 0, 0 })[0]}");
            Debug.Log($"Expected: 1 | Predicted: {net.Forward(new double[] { 0, 0, 1 })[0]}");
            Debug.Log($"Expected: 1 | Predicted: {net.Forward(new double[] { 0, 1, 0 })[0]}");
            Debug.Log($"Expected: 0 | Predicted: {net.Forward(new double[] { 0, 1, 1 })[0]}");
            Debug.Log($"Expected: 1 | Predicted: {net.Forward(new double[] { 1, 0, 0 })[0]}");
            Debug.Log($"Expected: 0 | Predicted: {net.Forward(new double[] { 1, 0, 1 })[0]}");
            Debug.Log($"Expected: 0 | Predicted: {net.Forward(new double[] { 1, 1, 0 })[0]}");
            Debug.Log($"Expected: 1 | Predicted: {net.Forward(new double[] { 1, 1, 1 })[0]}");

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

            Debug.Log($"Accuracy: {net.Test(inputs, outputs)}");
            Debug.Log(net);

            Destroy(gameObject);
        }
    }
}