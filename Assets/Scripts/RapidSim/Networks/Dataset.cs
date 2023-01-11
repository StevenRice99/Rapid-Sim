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
            Debug.Log($"Dataset {name} | {Size} Points");
        }

        public void CheckData()
        {
            if (Size == 0)
            {
                return;
            }

            double[] inputsAverage = new double[dataPoints[0].inputs.Length];
            double[] inputsLowest = new double[dataPoints[0].inputs.Length];
            double[] inputsHighest = new double[dataPoints[0].inputs.Length];

            for (int i = 0; i < inputsAverage.Length; i++)
            {
                inputsAverage[i] = 0;
                inputsLowest[i] = dataPoints[0].inputs[i];
                inputsHighest[i] = dataPoints[0].inputs[i];
            }
            
            double[] outputsAverage = new double[dataPoints[0].outputs.Length];
            double[] outputsLowest = new double[dataPoints[0].outputs.Length];
            double[] outputsHighest = new double[dataPoints[0].outputs.Length];

            for (int i = 0; i < outputsAverage.Length; i++)
            {
                outputsAverage[i] = 0;
                outputsLowest[i] = dataPoints[0].outputs[i];
                outputsHighest[i] = dataPoints[0].outputs[i];
            }

            for (int i = 0; i < Size; i++)
            {
                for (int j = 0; j < dataPoints[i].inputs.Length; j++)
                {
                    inputsAverage[j] += dataPoints[i].inputs[j];
                    if (dataPoints[i].inputs[j] < inputsLowest[j])
                    {
                        inputsLowest[j] = dataPoints[i].inputs[j];
                    }
                    else if (dataPoints[i].inputs[j] > inputsHighest[j])
                    {
                        inputsHighest[j] = dataPoints[i].inputs[j];
                    }
                }
                
                for (int j = 0; j < dataPoints[i].outputs.Length; j++)
                {
                    outputsAverage[j] += dataPoints[i].outputs[j];
                    if (dataPoints[i].outputs[j] < outputsLowest[j])
                    {
                        outputsLowest[j] = dataPoints[i].outputs[j];
                    }
                    else if (dataPoints[i].outputs[j] > outputsHighest[j])
                    {
                        outputsHighest[j] = dataPoints[i].outputs[j];
                    }
                }
            }
            
            for (int i = 0; i < inputsAverage.Length; i++)
            {
                inputsAverage[i] /= Size;
            }
            
            for (int i = 0; i < outputsAverage.Length; i++)
            {
                outputsAverage[i] /= Size;
            }

            string s = "Averages\nInputs";
            for (int i = 0; i < inputsAverage.Length; i++)
            {
                s += $"\n{i} = {inputsAverage[i]}";
            }

            s += "\nOutputs";
            for (int i = 0; i < outputsAverage.Length; i++)
            {
                s += $"\n{i} = {outputsAverage[i]}";
            }

            s += "\nLowest\nInputs";
            for (int i = 0; i < inputsLowest.Length; i++)
            {
                s += $"\n{i} = {inputsLowest[i]}";
            }

            s += "\nOutputs";
            for (int i = 0; i < outputsLowest.Length; i++)
            {
                s += $"\n{i} = {outputsLowest[i]}";
            }

            s += "\nHighest\nInputs";
            for (int i = 0; i < inputsHighest.Length; i++)
            {
                s += $"\n{i} = {inputsHighest[i]}";
            }

            s += "\nOutputs";
            for (int i = 0; i < outputsHighest.Length; i++)
            {
                s += $"\n{i} = {outputsHighest[i]}";
            }
            
            Debug.Log(s);
        }
    }
}