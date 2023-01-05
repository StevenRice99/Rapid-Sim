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
		
		[Tooltip("The number of epochs to stop training after testing accuracy has not improved.")]
		[Min(1)]
		public int limitWithoutImprovement = 100;
        
		[Tooltip("The current training epoch.")]
		[Min(0)]
		public int epoch;

		[SerializeField]
		[HideInInspector]
		private int currentWithoutImprovement;
		
		[SerializeField]
		[HideInInspector]
		private Layer[] layers;

		[SerializeField]
		[HideInInspector]
		private Layer[] bestLayers;

		[SerializeField]
		[HideInInspector]
		private EvaluationData bestAccuracy;

		private NetworkLearnData[] _batchLearnData;

		private void OnValidate()
		{
			if (epoch > limitWithoutImprovement)
			{
				epoch = limitWithoutImprovement;
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

			UpdateBestLayers();

			epoch = 0;
			currentWithoutImprovement = 0;
			
			bestAccuracy = new() {value = float.MaxValue};
			
			Debug.Log($"Network {name} Initialized.");
		}

		private void UpdateBestLayers()
		{
			bestLayers = new Layer[layers.Length];
			Array.Copy(layers, bestLayers, layers.Length);
		}

		// Run the inputs through the network to calculate the outputs
		public double[] Forward(double[] inputs)
		{
			return Forward(inputs, bestLayers);
		}

		// Run the inputs through the network to calculate the outputs
		private double[] Forward(double[] inputs, Layer[] architecture)
		{
			foreach (Layer layer in architecture)
			{
				inputs = layer.CalculateOutputs(inputs);
			}
			return inputs;
		}

		public bool Train(Dataset trainingDataset, Dataset testingDataset)
		{
			if (currentWithoutImprovement >= limitWithoutImprovement)
			{
				return false;
			}
			
			if (_batchLearnData == null || _batchLearnData.Length != trainingDataset.dataPoints.Count)
			{
				_batchLearnData = new NetworkLearnData[trainingDataset.dataPoints.Count];
				for (int i = 0; i < _batchLearnData.Length; i++)
				{
					_batchLearnData[i] = new(layers);
				}
			}

			System.Threading.Tasks.Parallel.For(0, trainingDataset.dataPoints.Count, i =>
			{
				UpdateGradients(trainingDataset.dataPoints[i], _batchLearnData[i]);
			});


			double currentLearningRate = 1.0 / (1.0 + learnRateDecay * epoch) * initialLearningRate;
			
			// Update weights and biases based on the calculated gradients
			for (int i = 0; i < layers.Length; i++)
			{
				layers[i].ApplyGradients(currentLearningRate / trainingDataset.dataPoints.Count, regularization, momentum);
			}
			
			epoch++;

			EvaluationData trainingAccuracy = Test(trainingDataset, layers);
			EvaluationData testingAccuracy = Test(testingDataset, layers);

			if (testingAccuracy.value < bestAccuracy.value)
			{
				UpdateBestLayers();
				bestAccuracy.value = testingAccuracy.value;
				currentWithoutImprovement = 0;
			}
			else
			{
				currentWithoutImprovement++;
			}
			
			Debug.Log($"{name} | Epoch {epoch} | Training = {trainingAccuracy} | Testing = {testingAccuracy}% | Best = {bestAccuracy}% | {currentWithoutImprovement} / {limitWithoutImprovement} Epochs without Improvement");
			
			return true;
		}

		public void Test(Dataset trainingDataset, Dataset testingDataset)
		{
			EvaluationData trainingAccuracy = Test(trainingDataset, bestLayers);
			EvaluationData testingAccuracy = Test(testingDataset, bestLayers);
			
			Debug.Log($"{name} | Best Training = {trainingAccuracy} | Best Testing = {testingAccuracy}%");
		}
		
		private EvaluationData Test(Dataset dataset, Layer[] architecture)
		{
			EvaluationData evalData = new();

			System.Threading.Tasks.Parallel.ForEach(dataset.dataPoints, data =>
			{
				double[] calculatedOutputs = Forward(data.inputs, architecture);

				double accuracy = 0;
				for (int i = 0; i < data.outputs.Length; i++)
				{
					accuracy += Math.Max(data.outputs[i], calculatedOutputs[i]) - Math.Min(data.outputs[i], calculatedOutputs[i]);
				}
				accuracy /= data.outputs.Length;

				lock (evalData)
				{
					evalData.value += accuracy;
				}
			});

			evalData.value /= dataset.dataPoints.Count;

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
	
	[Serializable]
	public class EvaluationData
	{
		public double value;

		public override string ToString()
		{
			return $"{(1 - value) * 100:F4}%";
		}
	}
}