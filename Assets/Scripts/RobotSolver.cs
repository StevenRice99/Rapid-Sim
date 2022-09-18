using System;
using System.Linq;
using UnityEngine;

public class RobotSolver : MonoBehaviour
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
    private int generations = 10;

    [SerializeField]
    [Min(1)]
    private int solvers = 20;

    [SerializeField]
    [Min(0)]
    private int elites = 5;

    [SerializeField]
    [Min(0)]
    private int randoms = 10;

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
        
        NeuralNetwork[] networks = new NeuralNetwork[solvers];
        for (int i = 0; i < solvers; i++)
        {
            networks[i] = new(new[] { joints.Length + 3, joints.Length + 3, joints.Length });
        }

        Vector3 targetPosition = target.position;
        float fitness = Vector3.Distance(tcp.position, targetPosition);
        
        float[] originals = new float[joints.Length];
        float[] inputs = new float[joints.Length + 3];

        inputs[0] = targetPosition.x - joints[0].position.x;
        inputs[1] = targetPosition.y - joints[0].position.y;
        inputs[2] = targetPosition.z - joints[0].position.z;
        
        for (int i = 0; i < joints.Length; i++)
        {
            originals[i] = joints[i].localEulerAngles.z;
            //inputs[i + 3] = joints[i].localEulerAngles.z;// / 360f;
            inputs[i + 3] = (joints[i].localEulerAngles.z - -1) / (1 - -1);
        }

        for (int i = 0; i < generations; i++)
        {
            for (int j = 0; j < solvers; j++)
            {
                float[] solution = networks[j].Activate(inputs);
                for (int k = 0; k < joints.Length; k++)
                {
                    joints[k].localEulerAngles = new(_originalAngles[k].x, _originalAngles[k].y, solution[k] * 180);
                }

                networks[j].Fitness = Vector3.Distance(tcp.position, targetPosition);
            }

            networks = networks.OrderBy(n => n.Fitness).ToArray();

            NeuralNetwork[] eliteNetworks = networks.Take(elites).ToArray();
            
            for (int j = elites; j < solvers; j++)
            {
                if (j < solvers - randoms)
                {
                    networks[j].Crossover(eliteNetworks[_random.NextInt(0, eliteNetworks.Length)], eliteNetworks[_random.NextInt(0, eliteNetworks.Length)], mutationChance);
                }
                else
                {
                    networks[j].Randomize();
                }
            }
        }

        if (networks[0].Fitness >= fitness)
        {
            for (int i = 0; i < joints.Length; i++)
            {
                joints[i].localEulerAngles = new(_originalAngles[i].x, _originalAngles[i].y, originals[i]);
            }
            
            return;
        }

        float[] best = networks[0].LastOutput();
        
        for (int i = 0; i < joints.Length; i++)
        {
            joints[i].localEulerAngles = new(_originalAngles[i].x, _originalAngles[i].y, best[i] * 180);
        }
    }
}