using System;

namespace RapidSim.Networks
{
    [Serializable]
    public struct DataPoint
    {
        public double[] inputs;
        public double[] outputs;
    }
}