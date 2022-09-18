using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class NeatGenome
{
    public List<NodeGene> NodeGenes;
    public List<ConGene> ConGenes;

    public NeatGenome()
    {
        NodeGenes = new();
        ConGenes = new();
    }

    public NeatGenome(List<NodeGene> nodeGens, List<ConGene> conGens)
    {
        NodeGenes = nodeGens;
        ConGenes = conGens;
    }

    public void MutateGenome()
    {
        float createEdgeChance = 80f;
        float createNodeChance = 8f;
        float chanceEdge = Random.Range(0f, 100f);
        float chanceNode = Random.Range(0f, 100f);

        if (chanceNode <= createNodeChance)
        {
            // Create Random New Node
            AddRandomNode();
        }
        if (chanceEdge <= createEdgeChance)
        {
            // Create Random New Edge
            AddRandomConnection();
        }
        // Mutate The Weights
        MutateWeights();
    }

    private void AddRandomNode()
    {
        if (ConGenes.Count != 0)
        {
            int randomCon = Random.Range(0, ConGenes.Count);
            ConGene mutatingCon = ConGenes[randomCon];
            int firstNode = mutatingCon.InputNode;
            int secondNode = mutatingCon.OutputNode;

            // Disable the mutating connection
            mutatingCon.IsActive = false;

            int newId = GetNextNodeId();
            
            NodeGene newNode = new(newId, NodeGene.GeneType.Hidden);
            NodeGenes.Add(newNode);

            int nextInovNum = GetNextInovNum();
            ConGene firstNewCon = new(firstNode, newNode.ID, 1f, true, nextInovNum);
            ConGenes.Add(firstNewCon);
            nextInovNum = GetNextInovNum();
            ConGene secondNewCon = new(newNode.ID, secondNode, mutatingCon.Weight, true, nextInovNum);
        }
    }

    private int GetNextNodeId()
    {
        int nextID = 0;
        foreach (NodeGene node in NodeGenes.Where(node => nextID <= node.ID))
        {
            nextID = node.ID;
        }
        nextID += 1;
        return nextID;
    }
    private bool AddRandomConnection()
    {
        int firstNode = Random.Range(0, NodeGenes.Count);
        int secondNode = Random.Range(0, NodeGenes.Count);
        NodeGene.GeneType firstGeneType = NodeGenes[firstNode].Type;
        NodeGene.GeneType secondGeneType = NodeGenes[secondNode].Type;


        if (firstGeneType == secondGeneType && firstGeneType != NodeGene.GeneType.Hidden)
        {
            return AddRandomConnection();
        }

        foreach(ConGene con in ConGenes)
        {
            if((firstNode == con.InputNode && secondNode == con.OutputNode) ||
                (secondNode == con.InputNode && firstNode == con.OutputNode))
            {
                return false;
            }
        }

        if (firstGeneType == NodeGene.GeneType.Output || (firstGeneType == NodeGene.GeneType.Hidden
            && secondGeneType == NodeGene.GeneType.Input))
        {
            int tmp = firstNode;
            firstNode = secondNode;
            secondNode = tmp;

            firstGeneType = NodeGenes[firstNode].Type;
            secondGeneType = NodeGenes[secondNode].Type;
        }

        int innov = GetNextInovNum();
        float weight = Random.Range(-1f, 1f);
        bool act = true;
        ConGene newCon = new(firstNode, secondNode, weight, act, innov); 
        ConGenes.Add(newCon);
        return true;
    }

    private int GetNextInovNum()
    {
        int nextInov = 0;
        foreach(ConGene con in ConGenes)
        {
            if(nextInov <= con.InnovNum)
            {
                nextInov = con.InnovNum;
            }
        }
        nextInov += 1;
        return nextInov;
    }
    private void MutateWeights()
    {
        float randomWeightChance = 30f;
        float perturbWeightChance = 90f;
        float chanceRandom = Random.Range(0f, 100f);
        float chancePerturb = Random.Range(0f, 100f);

        if (chanceRandom <= randomWeightChance)
        {
            // Randomize Single Weight
            RandomizeSingleWeight();
        }
        if (chancePerturb <= perturbWeightChance)
        {
            // Perturb Group of Weight
            PerturbWeights();
        }
    }

    private void RandomizeSingleWeight()
    {
        if (ConGenes.Count != 0)
        {
            int randomConIndex = Random.Range(0, ConGenes.Count);
            ConGene connection = ConGenes[randomConIndex];
            connection.Weight = Random.Range(-1f, 1f);
        }
    }

    private void PerturbWeights()
    {
        foreach (ConGene con in ConGenes)
        {
            con.Weight += Random.Range(-0.5f, 0.5f) * 0.5f;
        }
    }

}

public class NodeGene
{
    public int ID;
    
    public enum GeneType
    {
        Input, Output, Hidden
    }
    
    public GeneType Type;

    public NodeGene(int givenID, GeneType givenGeneType)
    {
        ID = givenID;
        Type = givenGeneType;
    }
}

public class ConGene
{
    public int InputNode;
    public int OutputNode;
    public float Weight;
    public bool IsActive;
    public int InnovNum;

    public ConGene(int inNode, int outNode, float wei, bool active, int inov)
    {
        InputNode = inNode;
        OutputNode = outNode;
        Weight = wei;
        IsActive = active;
        InnovNum = inov;
    }
}