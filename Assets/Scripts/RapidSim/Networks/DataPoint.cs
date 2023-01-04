using System;
using UnityEngine;

namespace RapidSim.Networks
{
    [Serializable]
    public struct DataPoint
    {
        [Tooltip("Inputs for training a neural network.")]
        public double[] inputs;
        
        [Tooltip("Outputs for training a neural network.")]
        public double[] outputs;

        public DataPoint(double[] inputs, double[] outputs)
        {
            this.inputs = inputs;
            this.outputs = outputs;
        }
    }
}