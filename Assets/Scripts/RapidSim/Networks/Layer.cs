using System;
using Unity.Mathematics;
using Random = Unity.Mathematics.Random;

namespace RapidSim.Networks
{
    [Serializable]
    public struct Layer
    {
        public double[] outputs;
        public WrappedArray[] weights;
        public double[] deltaBias;

        public double[] inputs;
        public int numberOfInputs;
        public int numberOfOutputs;
        public double[] bias;
        public WrappedArray[] deltaWeights;
        public WrappedArray[] momentumDeltaWeights;
        public WrappedArray[] velocityDeltaWeights;
        public double[] momentumDeltaBias;
        public double[] velocityDeltaBias;

        public int NumberOfParameters => numberOfInputs * numberOfOutputs + numberOfOutputs;

        public Layer(int numberOfInputs, int numberOfOutputs)
        {
            this.numberOfInputs = numberOfInputs;
            this.numberOfOutputs = numberOfOutputs;

            inputs = new double[this.numberOfInputs];
            outputs = new double[this.numberOfOutputs];

            weights = new WrappedArray[this.numberOfOutputs];
            deltaWeights = new WrappedArray[this.numberOfOutputs];
            momentumDeltaWeights = new WrappedArray[this.numberOfOutputs];
            velocityDeltaWeights = new WrappedArray[this.numberOfOutputs];
            for (int i = 0; i < this.numberOfOutputs; i++)
            {
                weights[i].data = new double[this.numberOfInputs];
                deltaWeights[i].data = new double[this.numberOfInputs];
                momentumDeltaWeights[i].data = new double[this.numberOfInputs];
                velocityDeltaWeights[i].data = new double[this.numberOfInputs];
            }

            bias = new double[this.numberOfOutputs];
            deltaBias = new double[this.numberOfOutputs];
            momentumDeltaBias = new double[this.numberOfOutputs];
            velocityDeltaBias = new double[this.numberOfOutputs];

            Reset();
        }

        public double[] Forward(double[] input)
        {
            for (int i = 0; i < inputs.Length; i++)
            {
                inputs[i] = input[i];
            }

            for (int i = 0; i < numberOfOutputs; i++)
            {
                outputs[i] = bias[i];
                for (int j = 0; j < numberOfInputs; j++)
                {
                    outputs[i] += inputs[j] * weights[i].data[j];
                }

                outputs[i] = Activation(outputs[i]);
            }

            return outputs;
        }

        public void BackwardOutput(double[] expected)
        {
            for (int i = 0; i < numberOfOutputs; i++)
            {
                deltaBias[i] = (outputs[i] - expected[i]) * ActivationDerivative(outputs[i]);
            
                for (int j = 0; j < numberOfInputs; j++)
                {
                    deltaWeights[i].data[j] = deltaBias[i] * inputs[j];
                }
            }
        }

        public void BackwardHidden(double[] gammaForward, WrappedArray[] weightsForward)
        {
            for (int i = 0; i < numberOfOutputs; i++)
            {
                deltaBias[i] = 0;

                for (int j = 0; j < gammaForward.Length; j++)
                {
                    deltaBias[i] += gammaForward[j] * weightsForward[j].data[i];
                }

                deltaBias[i] *= ActivationDerivative(outputs[i]);
            
                for (int j = 0; j < numberOfInputs; j++)
                {
                    deltaWeights[i].data[j] = deltaBias[i] * inputs[j];
                }
            }
        }

        public void Optimize(int step, double eta, double beta1, double beta2, double epsilon)
        {
            for (int i = 0; i < numberOfOutputs; i++)
            {
                for (int j = 0; j < numberOfInputs; j++)
                {
                    momentumDeltaWeights[i].data[j] = beta1 * momentumDeltaWeights[i].data[j] + (1 - beta1) * deltaWeights[i].data[j];
                    velocityDeltaWeights[i].data[j] = beta2 * velocityDeltaWeights[i].data[j] + (1 - beta2) * math.pow(deltaWeights[i].data[j], 2);
            
                    double momentumDeltaWeightCorrection = momentumDeltaWeights[i].data[j] / (1 - math.pow(beta1, step));
                    double velocityDeltaWeightCorrection = velocityDeltaWeights[i].data[j] / (1 - math.pow(beta2, step));
                
                    weights[i].data[j] -= eta * (momentumDeltaWeightCorrection / (math.sqrt(velocityDeltaWeightCorrection) + epsilon));
                }
            
                momentumDeltaBias[i] = beta1 * momentumDeltaBias[i] + (1 - beta1) * deltaBias[i];
                velocityDeltaBias[i] = beta2 * velocityDeltaBias[i] + (1 - beta2) * deltaBias[i];
            
                double momentumDeltaBiaCorrection = momentumDeltaBias[i] / (1 - math.pow(beta1, step));
                double velocityDeltaBiaCorrection = velocityDeltaBias[i] / (1 - math.pow(beta2, step));

                deltaBias[i] -= eta * (momentumDeltaBiaCorrection / (math.sqrt(velocityDeltaBiaCorrection) + epsilon));
            }
        }

        public void Reset()
        {
            uint seed = (uint) DateTime.UtcNow.Ticks;
            if (seed == 0)
            {
                seed = 1;
            }

            Random random = new(seed);

            for (int i = 0; i < numberOfOutputs; i++)
            {
                for (int j = 0; j < numberOfInputs; j++)
                {
                    weights[i].data[j] = random.NextFloat(-0.5f, 0.5f);
                    momentumDeltaWeights[i].data[j] = 0;
                    velocityDeltaWeights[i].data[j] = 0;
                }
            
                bias[i] = random.NextFloat(-0.5f, 0.5f);
                momentumDeltaBias[i] = 0;
                velocityDeltaBias[i] = 0;
            }
        }

        public void ResetOptimization()
        {
            for (int i = 0; i < numberOfOutputs; i++)
            {
                for (int j = 0; j < numberOfInputs; j++)
                {
                    momentumDeltaWeights[i].data[j] = 0;
                    velocityDeltaWeights[i].data[j] = 0;
                }
            
                momentumDeltaBias[i] = 0;
                velocityDeltaBias[i] = 0;
            }
        }

        private static double Activation(double value)
        {
            return math.tanh(value);
        }

        private static double ActivationDerivative(double value)
        {
            return 1 - value * value;
        }

        public override string ToString()
        {
            return $"Inputs: {inputs.Length} | Outputs: {outputs.Length} | Parameters: {NumberOfParameters}";
        }
    }
}