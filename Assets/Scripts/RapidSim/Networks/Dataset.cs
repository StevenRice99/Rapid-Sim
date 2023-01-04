using System.Collections.Generic;
using UnityEngine;

namespace RapidSim.Networks
{
    [CreateAssetMenu(fileName = "Dataset", menuName = "Dataset", order = 1)]
    public class Dataset : ScriptableObject
    {
        [Tooltip("What size to cap the dataset at during generation.")]
        [Min(1)]
        public int maxSize = 1000;
        
        [Tooltip("The data points which this dataset consists of.")]
        public List<DataPoint> dataPoints;

        public int Size => dataPoints.Count;

        public bool Complete => dataPoints.Count >= maxSize;

        public void Add(double[] inputs, double[] outputs)
        {
            dataPoints.Add(new(inputs, outputs));
        }
    }
}