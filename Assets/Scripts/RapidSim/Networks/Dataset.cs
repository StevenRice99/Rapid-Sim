using System.Collections.Generic;
using UnityEngine;

namespace RapidSim.Networks
{
    [CreateAssetMenu(fileName = "Dataset", menuName = "Dataset", order = 1)]
    public class Dataset : ScriptableObject
    {
        [Min(1)]
        public int maxSize = 1000;
        
        public List<DataPoint> dataPoints;

        public bool Add(double[] inputs, double[] outputs)
        {
            if (dataPoints.Count >= maxSize)
            {
                return false;
            }
            
            dataPoints.Add(new() {inputs = inputs, outputs = outputs});
            return true;
        }
    }
}