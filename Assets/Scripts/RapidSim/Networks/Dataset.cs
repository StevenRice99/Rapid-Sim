using System.Collections.Generic;
using UnityEngine;

namespace RapidSim.Networks
{
    [CreateAssetMenu(fileName = "Dataset", menuName = "Dataset", order = 1)]
    public class Dataset : ScriptableObject
    {
        [HideInInspector]
        [Tooltip("The number of data points.")]
        [SerializeField]
        private int size;
        
        [Tooltip("The data points of the dataset.")]
        [SerializeField]
        private List<DataPoint> dataPoints;

        public List<DataPoint> DataPoints => dataPoints;

        public int Size => size;

        public void Add(double[] inputs, double[] outputs)
        {
            dataPoints.Add(new(inputs, outputs));
            OnValidate();
            Debug.Log($" Dataset {name} | {size} Points");
        }

        private void OnValidate()
        {
            size = dataPoints.Count;
        }
    }
}