using System;
using System.Collections.Generic;
using BioIK.Setup;
using UnityEngine;

namespace BioIK.Helpers
{
	public class Model
	{
		//Reference to the character
		private readonly BioIK _character;

		//Reference to root
		private readonly BioSegment _root;

		//Offset to world
		private double _opx, _opy, _opz;
		private double _orx, _ory, _orz, _orw;
		private double _osx, _osy, _osz;
		
		//Linked list of nodes in the model
		private Node[] _nodes = Array.Empty<Node>();

		//Global pointers to the IK setup
		public MotionPtr[] MotionPointers = Array.Empty<MotionPtr>();
		private ObjectivePtr[] _objectivePointers = Array.Empty<ObjectivePtr>();

		//Assigned Configuration
		private readonly double[] _configuration;
		private readonly double[] _gradient;
		private readonly double[] _losses;

		//Simulated Configuration
		private readonly double[] _px;
		private readonly double[] _py;
		private readonly double[] _pz;
		private readonly double[] _rx;
		private readonly double[] _ry;
		private readonly double[] _rz;
		private readonly double[] _rw;
		private readonly double[] _simulatedLosses;

		//Degree of Freedom
		private readonly int _doF;

		public Model(BioIK character)
		{
			_character = character;

			//Set Root
			_root = _character.FindSegment(_character.transform);

			//Create Root
			AddNode(_root);
			
			//Build Model
			BioObjective[] objectives = CollectObjectives(_root, new());
			for (int i = 0; i < objectives.Length; i++)
			{
				List<BioSegment> chain = _character.GetChain(objectives[i].segment);
				for (int j = 1; j < chain.Count; j++)
				{
					AddNode(chain[j]);
				}
			}

			//Assign DoF
			_doF = MotionPointers.Length;

			//Initialise arrays for single transform modifications
			for (int i = 0; i < _nodes.Length; i++)
			{
				_nodes[i].ObjectiveImpacts = new bool[_objectivePointers.Length];
			}
			_px = new double[_objectivePointers.Length];
			_py = new double[_objectivePointers.Length];
			_pz = new double[_objectivePointers.Length];
			_rx = new double[_objectivePointers.Length];
			_ry = new double[_objectivePointers.Length];
			_rz = new double[_objectivePointers.Length];
			_rw = new double[_objectivePointers.Length];
			_configuration = new double[MotionPointers.Length];
			_gradient = new double[MotionPointers.Length];
			_losses = new double[_objectivePointers.Length];
			_simulatedLosses = new double[_objectivePointers.Length];

			//Assigns references to all objective nodes that are affected by a parenting node
			for (int i = 0; i < _objectivePointers.Length; i++)
			{
				Node node = _objectivePointers[i].Node;
				while (node != null)
				{
					node.ObjectiveImpacts[i] = true;
					node = node.Parent;
				}
			}

			Refresh();
		}

		public int GetDoF()
		{
			return _doF;
		}

		public BioIK GetCharacter()
		{
			return _character;
		}

		public void Refresh()
		{
			//Updates configuration
			for (int i = 0; i < _configuration.Length; i++)
			{
				_configuration[i] = MotionPointers[i].Motion.GetTargetValue(true);
			}

			//Update offset from world to root
			if (_root.transform.root == _character.transform)
			{
				_opx = _opy = _opz = _orx = _ory = _orz = 0.0;
				_orw = _osx = _osy = _osz = 1.0;
			}
			else
			{
				Transform parent = _root.transform.parent;
				Vector3 p = parent.position;
				Quaternion r = parent.rotation;
				Vector3 s = parent.lossyScale;
				_opx = p.x; _opy = p.y; _opz = p.z;
				_orx = r.x; _ory = r.y; _orz = r.z; _orw = r.w;
				_osx = s.x; _osy = s.y; _osz = s.z;
			}

			//Updates the nodes
			_nodes[0].Refresh();
		}

		public void CopyFrom(Model model)
		{
			_opx = model._opx;
			_opy = model._opy;
			_opz = model._opz;
			_orx = model._orx;
			_ory = model._ory;
			_orz = model._orz;
			_orw = model._orw;
			_osx = model._osx;
			_osy = model._osy;
			_osz = model._osz;
			for (int i = 0; i < _doF; i++)
			{
				_configuration[i] = model._configuration[i];
				_gradient[i] = model._gradient[i];
			}
			for (int i = 0; i < _objectivePointers.Length; i++)
			{
				_px[i] = model._px[i];
				_py[i] = model._py[i];
				_pz[i] = model._pz[i];
				_rx[i] = model._rx[i];
				_ry[i] = model._ry[i];
				_rz[i] = model._rz[i];
				_rw[i] = model._rw[i];
				_losses[i] = model._losses[i];
				_simulatedLosses[i] = model._simulatedLosses[i];
			}
			for (int i = 0; i < _nodes.Length; i++)
			{
				_nodes[i].Wpx = model._nodes[i].Wpx;
				_nodes[i].Wpy = model._nodes[i].Wpy;
				_nodes[i].Wpz = model._nodes[i].Wpz;
				_nodes[i].Wrx = model._nodes[i].Wrx;
				_nodes[i].Wry = model._nodes[i].Wry;
				_nodes[i].Wrz = model._nodes[i].Wrz;
				_nodes[i].Wrw = model._nodes[i].Wrw;
				_nodes[i].Wsx = model._nodes[i].Wsx;
				_nodes[i].Wsy = model._nodes[i].Wsy;
				_nodes[i].Wsz = model._nodes[i].Wsz;

				_nodes[i].Lpx = model._nodes[i].Lpx;
				_nodes[i].Lpy = model._nodes[i].Lpy;
				_nodes[i].Lpz = model._nodes[i].Lpz;
				_nodes[i].Lrx = model._nodes[i].Lrx;
				_nodes[i].Lry = model._nodes[i].Lry;
				_nodes[i].Lrz = model._nodes[i].Lrz;
				_nodes[i].Lrw = model._nodes[i].Lrw;
				
				_nodes[i].XValue = model._nodes[i].XValue;
				_nodes[i].YValue = model._nodes[i].YValue;
				_nodes[i].ZValue = model._nodes[i].ZValue;
			}
		}

		//Computes the loss as the RMSE over all objectives
		public double ComputeLoss(double[] configuration)
		{
			ForwardKinematics(configuration);
			double loss = 0.0;
			for (int i = 0; i < _objectivePointers.Length; i++)
			{
				Node node = _objectivePointers[i].Node;
				_losses[i] = _objectivePointers[i].Objective.ComputeLoss(node.Wpx, node.Wpy, node.Wpz, node.Wrx, node.Wry, node.Wrz, node.Wrw);
				loss += _losses[i];
			}
			return Math.Sqrt(loss / _objectivePointers.Length);
		}

		//Computes the gradient
		public double[] ComputeGradient(double[] configuration, double resolution)
		{
			double oldLoss = ComputeLoss(configuration);
			for (int j = 0; j < _doF; j++)
			{
				_configuration[j] += resolution;
				MotionPointers[j].Node.SimulateModification(_configuration);
				_configuration[j] -= resolution;
				double newLoss = 0.0;
				for (int i = 0; i < _objectivePointers.Length; i++)
				{
					newLoss += _simulatedLosses[i];
				}
				newLoss = Math.Sqrt(newLoss / _objectivePointers.Length);
				_gradient[j] = (newLoss - oldLoss) / resolution;
			}
			return _gradient;
		}

		//Applies a forward kinematics pass to the model
		private void ForwardKinematics(double[] configuration)
		{
			for (int i = 0; i < _configuration.Length; i++)
			{
				_configuration[i] = configuration[i];
			}
			_nodes[0].FeedForwardConfiguration(configuration);
		}

		//Adds a segment node into the model
		private void AddNode(BioSegment segment)
		{
			if (FindNode(segment.transform) != null)
			{
				return;
			}

			Node node = new(this, FindNode(segment.transform.parent), segment);

			if (node.Joint != null)
			{
				if (node.Joint.GetDoF() == 0 || !node.Joint.enabled)
				{
					node.Joint = null;
				}
				else
				{
					if (node.Joint.x.IsEnabled())
					{
						MotionPtr motionPtr = new(node.Joint.x, node, MotionPointers.Length);
						Array.Resize(ref MotionPointers, MotionPointers.Length + 1);
						MotionPointers[^1] = motionPtr;
						node.XEnabled = true;
						node.XIndex = motionPtr.Index;
					}
					if (node.Joint.y.IsEnabled())
					{
						MotionPtr motionPtr = new(node.Joint.y, node, MotionPointers.Length);
						Array.Resize(ref MotionPointers, MotionPointers.Length + 1);
						MotionPointers[^1] = motionPtr;
						node.YEnabled = true;
						node.YIndex = motionPtr.Index;
					}
					if (node.Joint.z.IsEnabled())
					{
						MotionPtr motionPtr = new(node.Joint.z, node, MotionPointers.Length);
						Array.Resize(ref MotionPointers, MotionPointers.Length + 1);
						MotionPointers[^1] = motionPtr;
						node.ZEnabled = true;
						node.ZIndex = motionPtr.Index;
					}
				}
			}

			BioObjective[] objectives = segment.objectives;
			for (int i = 0; i < objectives.Length; i++)
			{
				if (objectives[i].enabled)
				{
					Array.Resize(ref _objectivePointers, _objectivePointers.Length + 1);
					_objectivePointers[^1] = new(objectives[i], node);
				}
			}

			Array.Resize(ref _nodes, _nodes.Length + 1);
			_nodes[^1] = node;
		}

		//Returns all objectives which are childs in the hierarcy, beginning from the root
		private static BioObjective[] CollectObjectives(BioSegment segment, List<BioObjective> objectives)
		{
			for (int i = 0; i < segment.objectives.Length; i++)
			{
				if (segment.objectives[i].enabled)
				{
					objectives.Add(segment.objectives[i]);
				}
			}
			for (int i = 0; i < segment.children.Length; i++)
			{
				CollectObjectives(segment.children[i], objectives);
			}
			return objectives.ToArray();
		}

		//Returns a node in the model
		private Node FindNode(Transform t)
		{
			for (int i=0; i<_nodes.Length; i++)
			{
				if (_nodes[i].Transform == t)
				{
					return _nodes[i];
				}
			}
			return null;
		}

		//Returns the pointer to the objective
		public ObjectivePtr FindObjectivePtr(BioObjective objective)
		{
			for (int i = 0; i < _objectivePointers.Length; i++)
			{
				if (_objectivePointers[i].Objective == objective)
				{
					return _objectivePointers[i];
				}
			}
			return null;
		}

		//Subclass representing the single nodes for the OFKT data structure.
		//Values are stored using primitive data types for faster access and efficient computation.
		public class Node
		{
			private readonly Model _model;							//Reference to the kinematic model
			public readonly Node Parent;							//Reference to the parent of this node
			private Node[] _children = Array.Empty<Node>();			//Reference to all child nodes
			public readonly Transform Transform;					//Reference to the transform
			public BioJoint Joint;									//Reference to the joint
			public readonly Transform[] Chain;

			public double Wpx, Wpy, Wpz;				//World position
			public double Wrx, Wry, Wrz, Wrw;			//World rotation
			public double Wsx, Wsy, Wsz;				//World scale
			public double Lpx, Lpy, Lpz;				//Local position
			public double Lrx, Lry, Lrz, Lrw;			//Local rotation

			public bool XEnabled;
			public bool YEnabled;
			public bool ZEnabled;
			public int XIndex = -1;
			public int YIndex = -1;
			public int ZIndex = -1;
			public double XValue;
			public double YValue;
			public double ZValue;
		
			public bool[] ObjectiveImpacts;				//Boolean values to represent which objective indices in the whole kinematic tree are affected

			//Setup for the node
			public Node(Model model, Node parent, BioSegment segment)
			{
				_model = model;
				Parent = parent;
				Parent?.AddChild(this);
				Transform = segment.transform;
				Joint = segment.joint;

				List<Transform> reverseChain = new() { Transform };
				Node p = parent;
				while (p != null)
				{
					reverseChain.Add(p.Transform);
					p = p.Parent;
				}
				reverseChain.Reverse();
				Chain = reverseChain.ToArray();
			}

			//Adds a child to this node
			private void AddChild(Node child)
			{
				Array.Resize(ref _children, _children.Length+1);
				_children[^1] = child;
			}

			//Recursively refreshes the current transform data
			public void Refresh()
			{
				//Local
				if (Joint == null)
				{
					Vector3 lp = Transform.localPosition;
					Quaternion lr = Transform.localRotation;
					Lpx = lp.x;
					Lpy = lp.y;
					Lpz = lp.z;
					Lrx = lr.x;
					Lry = lr.y;
					Lrz = lr.z;
					Lrw = lr.w;
				}
				else
				{
					XValue = Joint.x.GetTargetValue(true);
					YValue = Joint.y.GetTargetValue(true);
					ZValue = Joint.z.GetTargetValue(true);
					Joint.ComputeLocalTransformation(XValue, YValue, ZValue, out Lpx, out Lpy, out Lpz, out Lrx, out Lry, out Lrz, out Lrw);
				}
				Vector3 ws = Transform.lossyScale;
				Wsx = ws.x;
				Wsy = ws.y;
				Wsz = ws.z;

				//World
				ComputeWorldTransformation();

				//Feed Forward
				foreach (Node child in _children)
				{
					child.Refresh();
				}
			}

			//Updates local and world transform, and feeds the joint variable configuration forward to all children
			public void FeedForwardConfiguration(double[] configuration, bool updateWorld = false)
			{
				//Assume no local update is required
				bool updateLocal = false;

				if (XEnabled)
				{
					XValue = configuration[XIndex];
					updateLocal = true;
				}
				
				if (YEnabled)
				{
					YValue = configuration[YIndex];
					updateLocal = true;
				}
				
				if (ZEnabled)
				{
					ZValue = configuration[ZIndex];
					updateLocal = true;
				}
				
				//Only update local transformation if a joint value has changed
				if (updateLocal)
				{
					Joint.ComputeLocalTransformation(XValue, YValue, ZValue, out Lpx, out Lpy, out Lpz, out Lrx, out Lry, out Lrz, out Lrw);
					updateWorld = true;
				}

				//Only update world transformation if local transformation (in this or parent node) has changed
				if (updateWorld)
				{
					ComputeWorldTransformation();
				}

				//Feed forward the joint variable configuration
				foreach (Node child in _children)
				{
					child.FeedForwardConfiguration(configuration, updateWorld);
				}
			}

			//Simulates a single transform modification while leaving the whole data structure unchanged
			//Returns the resulting Cartesian posture transformations in the out values
			public void SimulateModification(double[] configuration)
			{
				double[] px=_model._px; double[] py=_model._py; double[] pz=_model._pz;
				double[] rx=_model._rx; double[] ry=_model._ry; double[] rz=_model._rz; double[] rw=_model._rw;
				for(int i=0; i<_model._objectivePointers.Length; i++) {
					Node node = _model._objectivePointers[i].Node;
					if (ObjectiveImpacts[i])
					{
						Joint.ComputeLocalTransformation(
							XEnabled ? configuration[XIndex] : XValue,
							YEnabled ? configuration[YIndex] : YValue, 
							ZEnabled ? configuration[ZIndex] : ZValue, 
							out double lpX, out double lpY, out double lpZ, out double lrX, out double lrY, out double lrZ, out double lrW
						);
						double localRx, localRy, localRz, localRw, localX, localY, localZ;
						if (Parent == null)
						{
							px[i] = _model._opx;
							py[i] = _model._opy;
							pz[i] = _model._opz;
							localRx = _model._orx;
							localRy = _model._ory;
							localRz = _model._orz;
							localRw = _model._orw;
							localX = _model._osx*lpX;
							localY = _model._osy*lpY;
							localZ = _model._osz*lpZ;
						}
						else
						{
							px[i] = Parent.Wpx;
							py[i] = Parent.Wpy;
							pz[i] = Parent.Wpz;
							localRx = Parent.Wrx;
							localRy = Parent.Wry;
							localRz = Parent.Wrz;
							localRw = Parent.Wrw;
							localX = Parent.Wsx*lpX;
							localY = Parent.Wsy*lpY;
							localZ = Parent.Wsz*lpZ;
						}
						double qx = localRx * lrW + localRy * lrZ - localRz * lrY + localRw * lrX;
						double qy = -localRx * lrZ + localRy * lrW + localRz * lrX + localRw * lrY;
						double qz = localRx * lrY - localRy * lrX + localRz * lrW + localRw * lrZ;
						double qw = -localRx * lrX - localRy * lrY - localRz * lrZ + localRw * lrW;
						double dot = Wrx*Wrx + Wry*Wry + Wrz*Wrz + Wrw*Wrw;
						double x = qx / dot; double y = qy / dot; double z = qz / dot; double w = qw / dot;
						qx = x * Wrw + y * -Wrz - z * -Wry + w * -Wrx;
						qy = -x * -Wrz + y * Wrw + z * -Wrx + w * -Wry;
						qz = x * -Wry - y * -Wrx + z * Wrw + w * -Wrz;
						qw = -x * -Wrx - y * -Wry - z * -Wrz + w * Wrw;
						px[i] +=
								+ 2.0 * ((0.5 - localRy * localRy - localRz * localRz) * localX + (localRx * localRy - localRw * localRz) * localY + (localRx * localRz + localRw * localRy) * localZ)
								+ 2.0 * ((0.5 - qy * qy - qz * qz) * (node.Wpx-Wpx) + (qx * qy - qw * qz) * (node.Wpy-Wpy) + (qx * qz + qw * qy) * (node.Wpz-Wpz));
						py[i] += 
								+ 2.0 * ((localRx * localRy + localRw * localRz) * localX + (0.5 - localRx * localRx - localRz * localRz) * localY + (localRy * localRz - localRw * localRx) * localZ)
								+ 2.0 * ((qx * qy + qw * qz) * (node.Wpx-Wpx) + (0.5 - qx * qx - qz * qz) * (node.Wpy-Wpy) + (qy * qz - qw * qx) * (node.Wpz-Wpz));
						pz[i] += 
								+ 2.0 * ((localRx * localRz - localRw * localRy) * localX + (localRy * localRz + localRw * localRx) * localY + (0.5 - (localRx * localRx + localRy * localRy)) * localZ)
								+ 2.0 * ((qx * qz - qw * qy) * (node.Wpx-Wpx) + (qy * qz + qw * qx) * (node.Wpy-Wpy) + (0.5 - qx * qx - qy * qy) * (node.Wpz-Wpz));
						rx[i] = qx * node.Wrw + qy * node.Wrz - qz * node.Wry + qw * node.Wrx;
						ry[i] = -qx * node.Wrz + qy * node.Wrw + qz * node.Wrx + qw * node.Wry;
						rz[i] = qx * node.Wry - qy * node.Wrx + qz * node.Wrw + qw * node.Wrz;
						rw[i] = -qx * node.Wrx - qy * node.Wry - qz * node.Wrz + qw * node.Wrw;
						_model._simulatedLosses[i] = _model._objectivePointers[i].Objective.ComputeLoss(px[i], py[i], pz[i], rx[i], ry[i], rz[i], rw[i]);
					}
					else
					{
						px[i] = node.Wpx;
						py[i] = node.Wpy;
						pz[i] = node.Wpz;
						rx[i] = node.Wrx;
						ry[i] = node.Wry;
						rz[i] = node.Wrz;
						rw[i] = node.Wrw;
						_model._simulatedLosses[i] = _model._losses[i];
					}
				}
			}

			//Computes the world transformation using the current joint variable configuration
			private void ComputeWorldTransformation()
			{
				double rx, ry, rz, rw, x, y, z;
				if (Parent == null)
				{
					Wpx = _model._opx;
					Wpy = _model._opy;
					Wpz = _model._opz;
					rx = _model._orx;
					ry = _model._ory;
					rz = _model._orz;
					rw = _model._orw;
					x = _model._osx*Lpx;
					y = _model._osy*Lpy;
					z = _model._osz*Lpz;
				}
				else
				{
					Wpx = Parent.Wpx;
					Wpy = Parent.Wpy;
					Wpz = Parent.Wpz;
					rx = Parent.Wrx;
					ry = Parent.Wry;
					rz = Parent.Wrz;
					rw = Parent.Wrw;
					x = Parent.Wsx*Lpx;
					y = Parent.Wsy*Lpy;
					z = Parent.Wsz*Lpz;
				}
				Wpx += 2.0 * ((0.5 - ry * ry - rz * rz) * x + (rx * ry - rw * rz) * y + (rx * rz + rw * ry) * z);
				Wpy += 2.0 * ((rx * ry + rw * rz) * x + (0.5 - rx * rx - rz * rz) * y + (ry * rz - rw * rx) * z);
				Wpz += 2.0 * ((rx * rz - rw * ry) * x + (ry * rz + rw * rx) * y + (0.5 - rx * rx - ry * ry) * z);
				Wrx = rx * Lrw + ry * Lrz - rz * Lry + rw * Lrx;
				Wry = -rx * Lrz + ry * Lrw + rz * Lrx + rw * Lry;
				Wrz = rx * Lry - ry * Lrx + rz * Lrw + rw * Lrz;
				Wrw = -rx * Lrx - ry * Lry - rz * Lrz + rw * Lrw;
			}
		}

		//Data class to store pointers to the objectives
		public class ObjectivePtr
		{
			public readonly BioObjective Objective;
			public readonly Node Node;

			public ObjectivePtr(BioObjective objective, Node node)
			{
				Objective = objective;
				Node = node;
			}
		}

		//Data class to store pointers to the joint motions
		public class MotionPtr
		{
			public readonly BioJoint.Motion Motion;
			public readonly Node Node;
			public readonly int Index;
			
			public MotionPtr(BioJoint.Motion motion, Node node, int index)
			{
				Motion = motion;
				Node = node;
				Index = index;
			}
		}
	}
}