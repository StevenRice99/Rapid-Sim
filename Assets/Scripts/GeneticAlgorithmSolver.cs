using System;
using System.Linq;
using UnityEngine;

public class GeneticAlgorithmSolver : MonoBehaviour
{
    [SerializeField]
    private bool solve;
    
    [SerializeField]
    private Transform[] joints;

    [SerializeField]
    private Transform tcp;

    [SerializeField]
    private Transform target;

    [SerializeField]
    [Min(1)]
    private int generations = 200;

    [SerializeField]
    [Min(1)]
    private int solvers = 200;

    [SerializeField]
    [Min(0)]
    private int elites = 5;

    [SerializeField]
    [Min(0)]
    private int randoms = 50;

    [SerializeField]
    [Range(float.Epsilon, 1)]
    private float mutationChance = 0.05f;

    private Vector2[] _originalAngles;

    private Unity.Mathematics.Random _random;

    private void Start()
    {
        uint random = (uint) DateTime.Now.Millisecond;
        if (random == 0)
        {
            random = 1;
        }

        _random = new(random);
        
        _originalAngles = new Vector2[joints.Length];
        for (int i = 0; i < joints.Length; i++)
        {
            _originalAngles[i] = new(joints[i].eulerAngles.x, joints[i].eulerAngles.y);
        }
    }

    private void Update()
    {
        if (!solve)
        {
            return;
        }

        solve = false;
        
        float[] originals = new float[joints.Length];
        
        for (int i = 0; i < joints.Length; i++)
        {
            originals[i] = joints[i].localEulerAngles.z;
        }

        IkGeneticAlgorithm[] geneticAlgorithms = new IkGeneticAlgorithm[solvers];
        for (int i = 0; i < solvers; i++)
        {
            geneticAlgorithms[i] = new(joints.Length);
        }

        Vector3 targetPosition = target.position;
        float fitness = Vector3.Distance(tcp.position, targetPosition);

        for (int i = 0; i < generations; i++)
        {
            for (int j = 0; j < solvers; j++)
            {
                for (int k = 0; k < joints.Length; k++)
                {
                    joints[k].localEulerAngles = new(_originalAngles[k].x, _originalAngles[k].y, geneticAlgorithms[j].JointValues[k]);
                }

                geneticAlgorithms[j].Fitness = Vector3.Distance(tcp.position, targetPosition);
            }

            geneticAlgorithms = geneticAlgorithms.OrderBy(n => n.Fitness).ToArray();

            IkGeneticAlgorithm[] eliteNetworks = geneticAlgorithms.Take(elites).ToArray();

            for (int j = elites; j < solvers; j++)
            {
                if (j < solvers - randoms)
                {
                    geneticAlgorithms[j].Crossover(eliteNetworks[_random.NextInt(0, eliteNetworks.Length)], eliteNetworks[_random.NextInt(0, eliteNetworks.Length)], mutationChance);
                }
                else
                {
                    geneticAlgorithms[j].Randomize();
                }
            }
        }

        if (geneticAlgorithms[0].Fitness >= fitness)
        {
            for (int i = 0; i < joints.Length; i++)
            {
                joints[i].localEulerAngles = new(_originalAngles[i].x, _originalAngles[i].y, originals[i]);
            }

            return;
        }

        for (int i = 0; i < joints.Length; i++)
        {
            joints[i].localEulerAngles = new(_originalAngles[i].x, _originalAngles[i].y, geneticAlgorithms[0].JointValues[i] * 180);
        }
    }
}