using System;

namespace RapidSim.Networks
{
    [Serializable]
    public struct DataPoint
    {
        public double[] inputs;
        public double[] outputs;

        public DataPoint(double[] inputs, double[] outputs)
        {
            this.inputs = inputs;
            this.outputs = outputs;
        }
    }
}