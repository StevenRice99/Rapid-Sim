using System;

public class IkGeneticAlgorithm
{
    public float[] JointValues { get; }

    public float Fitness;

    private Unity.Mathematics.Random _random;

    public IkGeneticAlgorithm(int joints)
    {
        uint random = (uint) DateTime.Now.Millisecond;
        if (random == 0)
        {
            random = 1;
        }

        _random = new(random);
        
        JointValues = new float[joints];
        Randomize();
    }

    public void Crossover(IkGeneticAlgorithm parent1, IkGeneticAlgorithm parent2, float mutationChance)
    {
        for (int i = 0; i < JointValues.Length; i++)
        {
            if (_random.NextFloat(0, 1) < mutationChance)
            {
                JointValues[i] += _random.NextFloat(-10, 10);
            }
            else
            {
                JointValues[i] = _random.NextBool() ? parent1.JointValues[i] : parent2.JointValues[i];
            }
        }
    }
    
    public void Randomize()
    {
        for (int i = 0; i < JointValues.Length; i++)
        {
            JointValues[i] = _random.NextFloat(0, 360);
        }
    }
}