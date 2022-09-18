using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class NeatNetwork
{
    public NeatGenome MyGenome;
    public List<Node> Nodes;
    public List<Node> InputNodes;
    public List<Node> OutputNodes;
    public List<Node> HiddenNodes;
    public List<Connection> Connections;
    public float Fitness;
    
    public NeatNetwork(int inp, int outp, int hid)
    {
        MyGenome = CreateInitialGenome(inp, outp, hid);
        Nodes = new();
        InputNodes = new();
        OutputNodes = new();
        HiddenNodes = new();
        Connections = new();
        CreateNetwork();
    }

    private NeatGenome CreateInitialGenome(int inp, int outp, int hid)
    {
        List<NodeGene> newNodeGenes = new();
        List<ConGene> newConGenes = new();
        int nodeId = 0;

        for (int i = 0; i < inp; i++)
        {
            NodeGene newNodeGene = new(nodeId, NodeGene.GeneType.Input);
            newNodeGenes.Add(newNodeGene);
            nodeId += 1;
        }

        for (int i = 0; i < outp; i++)
        {
            NodeGene newNodeGene = new(nodeId, NodeGene.GeneType.Output);
            newNodeGenes.Add(newNodeGene);
            nodeId += 1;
        }

        for (int i = 0; i < hid; i++)
        {
            NodeGene newNodeGene = new(nodeId, NodeGene.GeneType.Hidden);
            newNodeGenes.Add(newNodeGene);
            nodeId += 1;
        }

        NeatGenome newGenome = new(newNodeGenes, newConGenes);
        return newGenome;
    }

    private void CreateNetwork()
    {
        ResetNetwork();
        // Creation of Network Structure: Nodes
        foreach(NodeGene nodeGene in MyGenome.NodeGenes)
        {
            Node newNode = new(nodeGene.ID);
            Nodes.Add(newNode);
            if(nodeGene.Type == NodeGene.GeneType.Input)
            {
                InputNodes.Add(newNode);
            }
            else if(nodeGene.Type == NodeGene.GeneType.Hidden)
            {
                HiddenNodes.Add(newNode);
            }
            else if(nodeGene.Type == NodeGene.GeneType.Output)
            {
                OutputNodes.Add(newNode);
            }
        }

        // Creation of Network Structure: Edges
        foreach(ConGene conGene in MyGenome.ConGenes)
        {
            Connection newCon = new(conGene.InputNode, conGene.OutputNode, conGene.Weight, conGene.IsActive);
            Connections.Add(newCon);
        }

        // Creation of Network Structure: Node Neighbors
        foreach(Node node in Nodes)
        {
            foreach(Connection con in Connections)
            {
                if(con.InputNode == node.ID)
                {
                    node.OutputConnections.Add(con);
                }
                else if(con.OutputNode == node.ID)
                {
                    node.InputConnections.Add(con);
                }
            }
        }
    }

    private void ResetNetwork()
    {
        Nodes.Clear();
        InputNodes.Clear();
        OutputNodes.Clear();
        HiddenNodes.Clear();
        Connections.Clear();
    }
    public void Mutate()
    {
        MyGenome.MutateGenome();
        CreateNetwork();
    }
    // Main Driver Function for the NeuralNet
    public float[] Activate(float[] inputs)
    {
        float[] outputs = new float[OutputNodes.Count];
        for(int i = 0; i < InputNodes.Count; i++)
        {
            InputNodes[i].SetInputNodeValue(inputs[i]);
            InputNodes[i].FeedForwardValue();
            InputNodes[i].Value = 0;
        }
        for (int i = 0; i < HiddenNodes.Count; i++)
        {
            HiddenNodes[i].SetHiddenNodeValue();
            HiddenNodes[i].FeedForwardValue();
            HiddenNodes[i].Value = 0;
        }
        for(int i = 0; i < OutputNodes.Count; i++)
        {
            OutputNodes[i].SetOutputNodeValue();
            outputs[i] = OutputNodes[i].Value;
            OutputNodes[i].Value = 0;
        }

        return outputs;
    }

    public float[] LastOutput()
    {
        float[] outputs = new float[OutputNodes.Count];
        for(int i = 0; i < OutputNodes.Count; i++)
        {
            OutputNodes[i].SetOutputNodeValue();
            outputs[i] = OutputNodes[i].Value;
            OutputNodes[i].Value = 0;
        }

        return outputs;
    }
}

public class Node
{
    public int ID;
    public float Value;
    public List<Connection> InputConnections;
    public List<Connection> OutputConnections;

    public Node(int ident)
    {
        ID = ident;
        InputConnections = new();
        OutputConnections = new();
    }

    public void SetInputNodeValue(float val)
    {
        val = Sigmoid(val);
        Value = val;
    }
    public void SetHiddenNodeValue()
    {
        float val = 0;
        foreach (Connection con in InputConnections)
        {
            val +=  con.Weight*con.InputNodeValue;
        }
        Value = TanH(val);
    }
    public void SetOutputNodeValue()
    {
        float val = InputConnections.Sum(con => con.Weight * con.InputNodeValue);
        Value = TanH(val);
    }

    public void FeedForwardValue()
    {
        foreach (Connection con in OutputConnections)
        {
            con.InputNodeValue = Value;
        }
    }

    private static float Sigmoid(float x)
    {
        return 1 / (1 + Mathf.Exp(-x));
    }

    private static float TanH(float x)
    {
        return 2 / (1 + Mathf.Exp(-2*x)) - 1;
    }

    private float TanHMod1(float x)
    {
        return 2 / (1 + Mathf.Exp(-4*x)) - 1;
    }
}

public class Connection
{
    public int InputNode;
    public int OutputNode;
    public float Weight;
    public bool IsActive;
    public float InputNodeValue;
    public Connection(int inNode, int outNode, float wei, bool active)
    {
        InputNode = inNode;
        OutputNode = outNode;
        Weight = wei;
        IsActive = active;
    }
}
