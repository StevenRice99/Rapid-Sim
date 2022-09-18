using UnityEngine;

public class NeatSolver : MonoBehaviour
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
    private int generations = 50;
    
    [SerializeField]
    [Min(1)]
    private int solvers = 50;

    [SerializeField]
    [Min(0)]
    private int elites = 5;

    [SerializeField]
    [Min(0)]
    private int randoms = 10;
    
    private NeatNetwork[] _networks;

    private Vector2[] _originalAngles;
    
    private void Start()
    {
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
        
        _networks = new NeatNetwork[solvers];

        for (int i =0; i < solvers; i++)
        {
            _networks[i] = new(joints.Length,joints.Length,joints.Length);
        }

        MutatePopulation();
        
        Vector3 targetPosition = target.position;
        float fitness = Vector3.Distance(tcp.position, targetPosition);
        
        float[] originals = new float[joints.Length];
        float[] inputs = new float[joints.Length];
        
        for (int i = 0; i < joints.Length; i++)
        {
            originals[i] = joints[i].localEulerAngles.z;
            inputs[i] = joints[i].localEulerAngles.z / 360f;
        }

        for (int i = 0; i < generations; i++)
        {
            for (int j = 0; j < solvers; j++)
            {
                float[] solution = _networks[j].Activate(inputs);
                for (int k = 0; k < joints.Length; k++)
                {
                    joints[k].localEulerAngles = new(_originalAngles[k].x, _originalAngles[k].y, solution[k] * 360f);
                }

                _networks[j].Fitness = Vector3.Distance(tcp.position, targetPosition);
            }
            
            for (int j = 0; j < _networks.Length; j++)
            {
                for (int k = j; k < _networks.Length; k++)
                {
                    if (_networks[j].Fitness > _networks[k].Fitness)
                    {
                        (_networks[j], _networks[k]) = (_networks[k], _networks[j]);
                    }
                }
            }
        
            NeatNetwork[] newPopulation = new NeatNetwork[solvers];
            for (int j = 0; j < solvers - randoms; j++)
            {
                newPopulation[j] = _networks[j];
            }
        
            for (int j = solvers - randoms; j < solvers; j++)
            {
                newPopulation[j] = new(joints.Length,joints.Length,joints.Length);
            }
        
            _networks = newPopulation;
            MutatePopulation();
        }
        
        if (_networks[0].Fitness >= fitness)
        {
            for (int i = 0; i < joints.Length; i++)
            {
                joints[i].localEulerAngles = new(_originalAngles[i].x, _originalAngles[i].y, originals[i]);
            }
            
            return;
        }

        float[] best = _networks[0].LastOutput();
        
        for (int i = 0; i < joints.Length; i++)
        {
            joints[i].localEulerAngles = new(_originalAngles[i].x, _originalAngles[i].y, best[i] * 360f);
        }
    }

    private void MutatePopulation()
    {
        for (int i = elites; i < solvers; i++)
        {
            _networks[i].Mutate();
        }
    }
}