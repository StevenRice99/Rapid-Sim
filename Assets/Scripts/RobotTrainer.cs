using System;
using System.Linq;
using UnityEngine;
using Random = Unity.Mathematics.Random;

public class RobotTrainer : MonoBehaviour
{
    private RobotController _robotController;

    [SerializeField]
    [Min(1)]
    private int solvers = 50;

    [SerializeField]
    [Min(0)]
    private int elites = 5;
    
    [SerializeField]
    [Min(0)]
    private int randoms = 25;
    
    [SerializeField]
    [Range(0, 1)]
    private float mutationChance = 0.05f;

    private float _chainLength;

    private Random _random;
    
    private void Start()
    {
        _robotController = GetComponent<RobotController>();
        if (_robotController == null)
        {
            Destroy(this);
            return;
        }
        
        uint random = (uint) DateTime.Now.Millisecond;
        if (random == 0)
        {
            random = 1;
        }

        _random = new(random);

        for (int i = 1; i < _robotController.joints.Length; i++)
        {
            _chainLength += Vector3.Distance(_robotController.joints[i - 1].position, _robotController.joints[i].position);
        }
    }

    private void Update()
    {
        if (_robotController.robotData == null)
        {
            _robotController.robotData = ScriptableObject.CreateInstance<RobotData>();
        }

        if (_robotController.robotData.Data == null || _robotController.robotData.Data.Length != solvers)
        {
            Debug.LogWarning("WHAT");
            _robotController.robotData.Data = new NeuralNetwork[solvers];
            for (int i = 0; i < solvers; i++)
            {
                _robotController.robotData.Data[i] = new(new[] { _robotController.joints.Length + 3, _robotController.joints.Length + 3, _robotController.joints.Length });
            }
        }

        float[] startPosition = new float[_robotController.joints.Length];
        for (int j = 0; j < _robotController.joints.Length; j++)
        {
            startPosition[j] = _random.NextFloat(0, 360);
            _robotController.joints[j].localEulerAngles = new(_robotController.OriginalAngles[j].x, _robotController.OriginalAngles[j].y, startPosition[j]);
        }
        
        for (int j = 0; j < _robotController.joints.Length; j++)
        {
            _robotController.joints[j].localEulerAngles = new(_robotController.OriginalAngles[j].x, _robotController.OriginalAngles[j].y, _random.NextFloat(0, 360));
        }

        Vector3 endPosition = _robotController.tcp.position;

        for (int i = 0; i < _robotController.robotData.Data.Length; i++)
        {
            float[] inputs = new float[_robotController.joints.Length + 3];

            for (int j = 0; j < _robotController.joints.Length; j++)
            {
                _robotController.joints[j].localEulerAngles = new(_robotController.OriginalAngles[j].x, _robotController.OriginalAngles[j].y, startPosition[j]);
                inputs[j + 3] = _robotController.joints[j].localEulerAngles.z / 180 - 1;
            }
            
            Vector3 tcpPosition = _robotController.tcp.position;
            inputs[0] = (tcpPosition.x - _robotController.joints[0].position.x) / _chainLength;
            inputs[1] = (tcpPosition.y - _robotController.joints[0].position.y) / _chainLength;
            inputs[2] = (tcpPosition.z - _robotController.joints[0].position.z) / _chainLength;
                
            float[] solution = _robotController.robotData.Data[i].Activate(inputs);
            
            for (int j = 0; j < _robotController.joints.Length; j++)
            {
                _robotController.joints[j].localEulerAngles = new(_robotController.OriginalAngles[j].x, _robotController.OriginalAngles[j].y, solution[j] * 180);
            }

            _robotController.robotData.Data[i].Fitness = Vector3.Distance(_robotController.tcp.position, endPosition);
        }

        _robotController.robotData.Data = _robotController.robotData.Data.OrderBy(t => t.Fitness).ToArray();
        
        NeuralNetwork[] eliteNetworks = _robotController.robotData.Data.Take(elites).ToArray();
            
        for (int j = elites; j < solvers; j++)
        {
            if (j < solvers - randoms)
            {
                _robotController.robotData.Data[j].Crossover(eliteNetworks[_random.NextInt(0, eliteNetworks.Length)], eliteNetworks[_random.NextInt(0, eliteNetworks.Length)], mutationChance);
            }
            else
            {
                _robotController.robotData.Data[j].Randomize();
            }
        }
    }
}