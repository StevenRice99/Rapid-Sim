using System.Collections.Generic;
using UnityEngine;

namespace RapidSim.Networks
{
    [CreateAssetMenu(fileName = "Dataset", menuName = "Dataset", order = 1)]
    public class Dataset : ScriptableObject
    {
        [Tooltip("Dummy variable to ensure data is saved.")]
        [SerializeField]
        private bool dummy;
        
        [Tooltip("The data points of the dataset.")]
        [SerializeField]
        private List<DataPoint> dataPoints;

        public List<DataPoint> DataPoints => dataPoints;

        public int Size => dataPoints.Count;

        public void Add(double[] inputs, double[] outputs)
        {
            dataPoints.Add(new(inputs, outputs));
        }
    }
}