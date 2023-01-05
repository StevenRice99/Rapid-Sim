using System;
using UnityEngine;
using Random = System.Random;

#if UNITY_EDITOR
using UnityEditor;
#endif

namespace RapidSim.Networks
{
	public class NeuralNetwork : ScriptableObject
	{
		[Header("Network Architecture")]
		[Tooltip("The architecture of the neural network")]
		[SerializeField]
		private int[] layerSizes;

		[Header("Training Parameters")]
		[Tooltip("The learning rate during the first epoch.")]
		[Range(0, 1)]
		[SerializeField]
		private double initialLearningRate = 0.05;
		
		[Tooltip("How much to decay the learning rate every epoch.")]
		[Range(0, 1)]
		[SerializeField]
		private double learnRateDecay = 0.075;
		
		[Tooltip("The momentum to apply to the gradient descents of weights and biases.")]
		[Range(0, 1)]
		[SerializeField]
		private double momentum = 0.9;
		
		[Tooltip("The regularization to apply to the gradient descents of weights and biases.")]
		[Range(0, 1)]
		[SerializeField]
		private double regularization = 0.1;
		
		[Tooltip("The maximum number of epochs to perform training for.")]
		[Min(1)]
		public int epochs = 100;
        
		[Tooltip("The current training epoch.")]
		[Min(0)]
		public int currentEpoch;
		
		[SerializeField]
		[HideInInspector]
		private Layer[] layers;

		private NetworkLearnData[] _batchLearnData;

		private void OnValidate()
		{
			if (currentEpoch > epochs)
			{
				currentEpoch = epochs;
			}
            
			bool create = layers == null || layers.Length != layerSizes.Length - 1;

			if (!create)
			{
				for (int i = 0; i < layers.Length; i++)
				{
					if (layers[i].numNodesIn == layerSizes[i] && layers[i].numNodesOut == layerSizes[i + 1])
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

			Random rng = new();

			layers = new Layer[layerSizes.Length - 1];
			for (int i = 0; i < layers.Length; i++)
			{
				layers[i] = new(layerSizes[i], layerSizes[i + 1], rng);
			}

			currentEpoch = 0;
			
			Debug.Log($"Network {name} Initialized.");
		}

		// Run the inputs through the network to calculate the outputs
		public double[] Forward(double[] inputs)
		{
			foreach (Layer layer in layers)
			{
				inputs = layer.CalculateOutputs(inputs);
			}
			return inputs;
		}

		public bool Train(Dataset dataset)
		{
			if (currentEpoch >= epochs)
			{
				return false;
			}
			
			if (_batchLearnData == null || _batchLearnData.Length != dataset.dataPoints.Count)
			{
				_batchLearnData = new NetworkLearnData[dataset.dataPoints.Count];
				for (int i = 0; i < _batchLearnData.Length; i++)
				{
					_batchLearnData[i] = new(layers);
				}
			}

			System.Threading.Tasks.Parallel.For(0, dataset.dataPoints.Count, i =>
			{
				UpdateGradients(dataset.dataPoints[i], _batchLearnData[i]);
			});


			double currentLearningRate = 1.0 / (1.0 + learnRateDecay * currentEpoch) * initialLearningRate;
			
			// Update weights and biases based on the calculated gradients
			for (int i = 0; i < layers.Length; i++)
			{
				layers[i].ApplyGradients(currentLearningRate / dataset.dataPoints.Count, regularization, momentum);
			}
			
			currentEpoch++;
			
			return true;
		}
		
		public EvaluationData Test(Dataset dataset)
		{
			EvaluationData evalData = new();

			System.Threading.Tasks.Parallel.ForEach(dataset.dataPoints, data =>
			{
				double[] calculatedOutputs = Forward(data.inputs);

				double accuracy = 0;
				for (int i = 0; i < data.outputs.Length; i++)
				{
					accuracy += Math.Max(data.outputs[i], calculatedOutputs[i]) - Math.Min(data.outputs[i], calculatedOutputs[i]);
				}
				accuracy /= data.outputs.Length;

				lock (evalData)
				{
					evalData.accuracy += accuracy;
				}
			});

			evalData.accuracy /= dataset.dataPoints.Count;

			return evalData;
		}

		private void UpdateGradients(DataPoint dataPoint, NetworkLearnData learnData)
		{
			// Feed data through the network to calculate outputs.
			// Save all inputs/weightedinputs/activations along the way to use for backpropagation.
			double[] inputsToNextLayer = dataPoint.inputs;

			for (int i = 0; i < layers.Length; i++)
			{
				inputsToNextLayer = layers[i].CalculateOutputs(inputsToNextLayer, learnData.layerData[i]);
			}

			// -- Backpropagation --
			int outputLayerIndex = layers.Length - 1;
			Layer outputLayer = layers[outputLayerIndex];
			LayerLearnData outputLearnData = learnData.layerData[outputLayerIndex];

			// Update output layer gradients
			outputLayer.CalculateOutputLayerNodeValues(outputLearnData, dataPoint.outputs);
			outputLayer.UpdateGradients(outputLearnData);

			// Update all hidden layer gradients
			for (int i = outputLayerIndex - 1; i >= 0; i--)
			{
				LayerLearnData layerLearnData = learnData.layerData[i];
				Layer hiddenLayer = layers[i];

				hiddenLayer.CalculateHiddenLayerNodeValues(layerLearnData, layers[i + 1], learnData.layerData[i + 1].nodeValues);
				hiddenLayer.UpdateGradients(layerLearnData);
			}
		}

		public static bool Validate(Component attached, NeuralNetwork network, int requiredInputs, int requiredOutputs)
		{
			if (network == null)
			{
				Debug.LogError($"\"{attached.name}\" does not have a neural network attached to it.");
				return Stop(attached);
			}
            
			if (network.layers is {Length: 0})
			{
				Debug.LogError($"Neural network \"{network.name}\" attached to \"{attached.name}\" is not setup.");
				return Stop(attached);
			}

			if (network.layers[0].numNodesIn != requiredInputs)
			{
				Debug.LogError($"Neural network \"{network.name}\" attached to \"{attached.name}\" requires {requiredInputs} inputs but has {network.layers[0].numNodesIn} inputs.");
				return Stop(attached);
			}

			if (network.layers[^1].numNodesOut != requiredOutputs)
			{
				Debug.LogError($"Neural network \"{network.name}\" attached to \"{attached.name}\" requires {requiredOutputs} outputs but has {network.layers[^1].numNodesOut} outputs.");
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
	}

	public class NetworkLearnData
	{
		public readonly LayerLearnData[] layerData;

		public NetworkLearnData(Layer[] layers)
		{
			layerData = new LayerLearnData[layers.Length];
			for (int i = 0; i < layers.Length; i++)
			{
				layerData[i] = new(layers[i]);
			}
		}
	}

	public class LayerLearnData
	{
		public double[] inputs;
		public readonly double[] weightedInputs;
		public readonly double[] activations;
		public readonly double[] nodeValues;

		public LayerLearnData(Layer layer)
		{
			weightedInputs = new double[layer.numNodesOut];
			activations = new double[layer.numNodesOut];
			nodeValues = new double[layer.numNodesOut];
		}
	}
		
	public class EvaluationData
	{
		public double accuracy;

		public override string ToString()
		{
			return $"{accuracy * 100:F4}%";
		}
	}
}