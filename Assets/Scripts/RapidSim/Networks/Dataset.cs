using System.Collections.Generic;
using UnityEngine;

#if UNITY_EDITOR
using UnityEditor;
#endif

namespace RapidSim.Networks
{
    [CreateAssetMenu(fileName = "Dataset", menuName = "Dataset", order = 1)]
    public class Dataset : ScriptableObject
    {
        [Tooltip("The data points of the dataset.")]
        [SerializeField]
        private List<DataPoint> dataPoints;

        public List<DataPoint> DataPoints => dataPoints;

        public int Size => dataPoints.Count;

        public void Add(double[] inputs, double[] outputs)
        {
            dataPoints.Add(new(inputs, outputs));
#if UNITY_EDITOR
            EditorUtility.SetDirty(this);
#endif
            Debug.Log($" Dataset {name} | {Size} Points");
        }
    }
}