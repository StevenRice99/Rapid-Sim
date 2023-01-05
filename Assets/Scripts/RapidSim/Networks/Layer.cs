using System;
using Unity.Mathematics;
using Random = System.Random;

namespace RapidSim.Networks
{
	[Serializable]
	public class Layer
	{
		public int numNodesIn;
		public int numNodesOut;

		public double[] weights;
		public double[] biases;

		// Cost gradient with respect to weights and with respect to biases
		public double[] costGradientW;
		public double[] costGradientB;

		// Used for adding momentum to gradient descent
		public double[] weightVelocities;
		public double[] biasVelocities;

		// Create the layer
		public Layer(int numNodesIn, int numNodesOut, Random rng)
		{
			this.numNodesIn = numNodesIn;
			this.numNodesOut = numNodesOut;

			weights = new double[numNodesIn * numNodesOut];
			costGradientW = new double[weights.Length];
			biases = new double[numNodesOut];
			costGradientB = new double[biases.Length];

			weightVelocities = new double[weights.Length];
			biasVelocities = new double[biases.Length];

			for (int i = 0; i < weights.Length; i++)
			{
				weights[i] = RandomInNormalDistribution(rng, 0, 1) / math.sqrt(numNodesIn);
			}
		}

		// Calculate layer output activations
		public double[] CalculateOutputs(double[] inputs)
		{
			double[] weightedInputs = new double[numNodesOut];

			for (int nodeOut = 0; nodeOut < numNodesOut; nodeOut++)
			{
				double weightedInput = biases[nodeOut];

				for (int nodeIn = 0; nodeIn < numNodesIn; nodeIn++)
				{
					weightedInput += inputs[nodeIn] * GetWeight(nodeIn, nodeOut);
				}
				weightedInputs[nodeOut] = weightedInput;
			}

			// Apply activation function
			double[] activations = new double[numNodesOut];
			for (int outputNode = 0; outputNode < numNodesOut; outputNode++)
			{
				activations[outputNode] = Activate(weightedInputs, outputNode);
			}

			return activations;
		}

		// Calculate layer output activations and store inputs/weightedInputs/activations in the given learnData object
		public double[] CalculateOutputs(double[] inputs, LayerLearnData learnData)
		{
			learnData.inputs = inputs;

			for (int nodeOut = 0; nodeOut < numNodesOut; nodeOut++)
			{
				double weightedInput = biases[nodeOut];
				for (int nodeIn = 0; nodeIn < numNodesIn; nodeIn++)
				{
					weightedInput += inputs[nodeIn] * GetWeight(nodeIn, nodeOut);
				}
				learnData.weightedInputs[nodeOut] = weightedInput;
			}

			// Apply activation function
			for (int i = 0; i < learnData.activations.Length; i++)
			{
				learnData.activations[i] = Activate(learnData.weightedInputs, i);
			}

			return learnData.activations;
		}

		// Update weights and biases based on previously calculated gradients.
		// Also resets the gradients to zero.
		public void ApplyGradients(double learnRate, double regularization, double momentum)
		{
			double weightDecay = 1 - regularization * learnRate;

			for (int i = 0; i < weights.Length; i++)
			{
				double weight = weights[i];
				double velocity = weightVelocities[i] * momentum - costGradientW[i] * learnRate;
				weightVelocities[i] = velocity;
				weights[i] = weight * weightDecay + velocity;
				costGradientW[i] = 0;
			}


			for (int i = 0; i < biases.Length; i++)
			{
				double velocity = biasVelocities[i] * momentum - costGradientB[i] * learnRate;
				biasVelocities[i] = velocity;
				biases[i] += velocity;
				costGradientB[i] = 0;
			}
		}

		// Calculate the "node values" for the output layer. This is an array containing for each node:
		// the partial derivative of the cost with respect to the weighted input
		public void CalculateOutputLayerNodeValues(LayerLearnData layerLearnData, double[] expectedOutputs)
		{
			for (int i = 0; i < layerLearnData.nodeValues.Length; i++)
			{
				// Evaluate partial derivatives for current node: cost/activation & activation/weightedInput
				double costDerivative = CostDerivative(layerLearnData.activations[i], expectedOutputs[i]);
				double activationDerivative = Derivative(layerLearnData.weightedInputs, i);
				layerLearnData.nodeValues[i] = costDerivative * activationDerivative;
			}
		}

		// Calculate the "node values" for a hidden layer. This is an array containing for each node:
		// the partial derivative of the cost with respect to the weighted input
		public void CalculateHiddenLayerNodeValues(LayerLearnData layerLearnData, Layer oldLayer, double[] oldNodeValues)
		{
			for (int newNodeIndex = 0; newNodeIndex < numNodesOut; newNodeIndex++)
			{
				double newNodeValue = 0;
				for (int oldNodeIndex = 0; oldNodeIndex < oldNodeValues.Length; oldNodeIndex++)
				{
					// Partial derivative of the weighted input with respect to the input
					double weightedInputDerivative = oldLayer.GetWeight(newNodeIndex, oldNodeIndex);
					newNodeValue += weightedInputDerivative * oldNodeValues[oldNodeIndex];
				}
				newNodeValue *= Derivative(layerLearnData.weightedInputs, newNodeIndex);
				layerLearnData.nodeValues[newNodeIndex] = newNodeValue;
			}

		}

		public void UpdateGradients(LayerLearnData layerLearnData)
		{
			// Update cost gradient with respect to weights (lock for multithreading)
			lock (costGradientW)
			{
				for (int nodeOut = 0; nodeOut < numNodesOut; nodeOut++)
				{
					double nodeValue = layerLearnData.nodeValues[nodeOut];
					for (int nodeIn = 0; nodeIn < numNodesIn; nodeIn++)
					{
						// Evaluate the partial derivative: cost / weight of current connection
						double derivativeCostWrtWeight = layerLearnData.inputs[nodeIn] * nodeValue;
						// The costGradientW array stores these partial derivatives for each weight.
						// Note: the derivative is being added to the array here because ultimately we want
						// to calculate the average gradient across all the data in the training batch
						costGradientW[GetFlatWeightIndex(nodeIn, nodeOut)] += derivativeCostWrtWeight;
					}
				}
			}

			// Update cost gradient with respect to biases (lock for multithreading)
			lock (costGradientB)
			{
				for (int nodeOut = 0; nodeOut < numNodesOut; nodeOut++)
				{
					// Evaluate partial derivative: cost / bias
					double derivativeCostWrtBias = 1 * layerLearnData.nodeValues[nodeOut];
					costGradientB[nodeOut] += derivativeCostWrtBias;
				}
			}
		}

		public double GetWeight(int nodeIn, int nodeOut)
		{
			int flatIndex = nodeOut * numNodesIn + nodeIn;
			return weights[flatIndex];
		}

		public int GetFlatWeightIndex(int inputNeuronIndex, int outputNeuronIndex)
		{
			return outputNeuronIndex * numNodesIn + inputNeuronIndex;
		}

		private double RandomInNormalDistribution(Random rng, double mean, double standardDeviation)
		{
			double x1 = 1 - rng.NextDouble();
			double x2 = 1 - rng.NextDouble();

			double y1 = math.sqrt(-2.0 * math.log(x1)) * math.cos(2.0 * math.PI_DBL * x2);
			return y1 * standardDeviation + mean;
		}
	
		public double Activate(double[] inputs, int index)
		{
			return math.max(0, inputs[index]);
		}

		public double Derivative(double[] inputs, int index)
		{
			return inputs[index] > 0 ? 1 : 0;
		}
	
		public double CostFunction(double[] predictedOutputs, double[] expectedOutputs)
		{
			// cost is sum (for all x,y pairs) of: 0.5 * (x-y)^2
			double cost = 0;
			for (int i = 0; i < predictedOutputs.Length; i++)
			{
				double error = predictedOutputs[i] - expectedOutputs[i];
				cost += error * error;
			}
			return 0.5 * cost;
		}

		public double CostDerivative(double predictedOutput, double expectedOutput)
		{
			return predictedOutput - expectedOutput;
		}
	}
}