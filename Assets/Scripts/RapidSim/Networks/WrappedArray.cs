using System;
using UnityEngine;

namespace RapidSim.Networks
{
    [Serializable]
    public struct WrappedArray
    {
        [Tooltip("Array of values stored.")]
        public double[] data;
    }
}