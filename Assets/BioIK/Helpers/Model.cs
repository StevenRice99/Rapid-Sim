using System;
using System.Collections.Generic;
using BioIK.Setup;
using UnityEngine;

namespace BioIK.Helpers
{
	public class Model
	{
		private readonly BioIK _bioRobot;

		// Reference to root
		private readonly BioSegment _root;

		// Offset to world
		private double _opx, _opy, _opz;
		private double _orx, _ory, _orz, _orw;
		private double _osx, _osy, _osz;
		
		// Linked list of nodes in the model
		private Node[] _nodes = Array.Empty<Node>();

		//Global pointers to the IK setup
		public MotionPtr[] motionPointers = Array.Empty<MotionPtr>();
		private ObjectivePtr _objectivePointer;

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

		public Model(BioIK bioRobot)
		{
			_bioRobot = bioRobot;

			//Set Root
			_root = _bioRobot.FindSegment(_bioRobot.transform);

			//Create Root
			AddNode(_root);
			
			//Build Model
			BioObjective[] objectives = CollectObjectives(_root, new());
			for (int i = 0; i < objectives.Length; i++)
			{
				List<BioSegment> chain = _bioRobot.GetChain(objectives[i].segment);
				for (int j = 1; j < chain.Count; j++)
				{
					AddNode(chain[j]);
				}
			}

			//Assign DoF
			_doF = motionPointers.Length;

			//Initialise arrays for single transform modifications
			for (int i = 0; i < _nodes.Length; i++)
			{
				_nodes[i].objectiveImpacts = new bool[1];
			}
			_px = new double[1];
			_py = new double[1];
			_pz = new double[1];
			_rx = new double[1];
			_ry = new double[1];
			_rz = new double[1];
			_rw = new double[1];
			_configuration = new double[motionPointers.Length];
			_gradient = new double[motionPointers.Length];
			_losses = new double[1];
			_simulatedLosses = new double[1];

			//Assigns references to all objective nodes that are affected by a parenting node
			if (_objectivePointer != null)
			{
				Node node = _objectivePointer.node;
				while (node != null)
				{
					node.objectiveImpacts[0] = true;
					node = node.parent;
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
			return _bioRobot;
		}

		public void Refresh()
		{
			//Updates configuration
			for (int i = 0; i < _configuration.Length; i++)
			{
				_configuration[i] = motionPointers[i].motion.GetTargetValue(true);
			}

			//Update offset from world to root
			if (_root.transform.root == _bioRobot.transform)
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
			_px[0] = model._px[0];
			_py[0] = model._py[0];
			_pz[0] = model._pz[0];
			_rx[0] = model._rx[0];
			_ry[0] = model._ry[0];
			_rz[0] = model._rz[0];
			_rw[0] = model._rw[0];
			_losses[0] = model._losses[0];
			_simulatedLosses[0] = model._simulatedLosses[0];
			for (int i = 0; i < _nodes.Length; i++)
			{
				_nodes[i].wpx = model._nodes[i].wpx;
				_nodes[i].wpy = model._nodes[i].wpy;
				_nodes[i].wpz = model._nodes[i].wpz;
				_nodes[i].wrx = model._nodes[i].wrx;
				_nodes[i].wry = model._nodes[i].wry;
				_nodes[i].wrz = model._nodes[i].wrz;
				_nodes[i].wrw = model._nodes[i].wrw;
				_nodes[i].wsx = model._nodes[i].wsx;
				_nodes[i].wsy = model._nodes[i].wsy;
				_nodes[i].wsz = model._nodes[i].wsz;

				_nodes[i].lpx = model._nodes[i].lpx;
				_nodes[i].lpy = model._nodes[i].lpy;
				_nodes[i].lpz = model._nodes[i].lpz;
				_nodes[i].lrx = model._nodes[i].lrx;
				_nodes[i].lry = model._nodes[i].lry;
				_nodes[i].lrz = model._nodes[i].lrz;
				_nodes[i].lrw = model._nodes[i].lrw;
				
				_nodes[i].xValue = model._nodes[i].xValue;
				_nodes[i].yValue = model._nodes[i].yValue;
				_nodes[i].zValue = model._nodes[i].zValue;
			}
		}

		//Computes the loss as the RMSE over all objectives
		public double ComputeLoss(double[] configuration)
		{
			ForwardKinematics(configuration);
			Node node = _objectivePointer.node;
			_losses[0] = _objectivePointer.objective.ComputeLoss(node.wpx, node.wpy, node.wpz, node.wrx, node.wry, node.wrz, node.wrw);
			return Math.Sqrt(_losses[0]);
		}

		//Computes the gradient
		public double[] ComputeGradient(double[] configuration, double resolution)
		{
			double oldLoss = ComputeLoss(configuration);
			for (int j = 0; j < _doF; j++)
			{
				_configuration[j] += resolution;
				motionPointers[j].node.SimulateModification(_configuration);
				_configuration[j] -= resolution;
				_gradient[j] = (Math.Sqrt(_simulatedLosses[0]) - oldLoss) / resolution;
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

			if (node.joint != null)
			{
				if (node.joint.GetDoF() == 0 || !node.joint.enabled)
				{
					node.joint = null;
				}
				else
				{
					if (node.joint.x.IsEnabled())
					{
						MotionPtr motionPtr = new(node.joint.x, node, motionPointers.Length);
						Array.Resize(ref motionPointers, motionPointers.Length + 1);
						motionPointers[^1] = motionPtr;
						node.xEnabled = true;
						node.xIndex = motionPtr.index;
					}
					if (node.joint.y.IsEnabled())
					{
						MotionPtr motionPtr = new(node.joint.y, node, motionPointers.Length);
						Array.Resize(ref motionPointers, motionPointers.Length + 1);
						motionPointers[^1] = motionPtr;
						node.yEnabled = true;
						node.yIndex = motionPtr.index;
					}
					if (node.joint.z.IsEnabled())
					{
						MotionPtr motionPtr = new(node.joint.z, node, motionPointers.Length);
						Array.Resize(ref motionPointers, motionPointers.Length + 1);
						motionPointers[^1] = motionPtr;
						node.zEnabled = true;
						node.zIndex = motionPtr.index;
					}
				}
			}

			BioObjective objective = segment.objective;
			if (objective != null && objective.enabled)
			{
				_objectivePointer = new(objective, node);
			}

			Array.Resize(ref _nodes, _nodes.Length + 1);
			_nodes[^1] = node;
		}

		//Returns all objectives which are childs in the hierarcy, beginning from the root
		private static BioObjective[] CollectObjectives(BioSegment segment, List<BioObjective> objectives)
		{
			if (segment.objective != null && segment.objective.enabled)
			{
				objectives.Add(segment.objective);
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
				if (_nodes[i].transform == t)
				{
					return _nodes[i];
				}
			}
			return null;
		}

		//Returns the pointer to the objective
		public ObjectivePtr FindObjectivePtr(BioObjective objective)
		{
			return _objectivePointer.objective == objective ? _objectivePointer : null;
		}

		//Subclass representing the single nodes for the OFKT data structure.
		//Values are stored using primitive data types for faster access and efficient computation.
		public class Node
		{
			private readonly Model _model;							//Reference to the kinematic model
			public readonly Node parent;							//Reference to the parent of this node
			private Node[] _children = Array.Empty<Node>();			//Reference to all child nodes
			public readonly Transform transform;					//Reference to the transform
			public BioJoint joint;									//Reference to the joint
			public readonly Transform[] chain;

			public double wpx, wpy, wpz;				//World position
			public double wrx, wry, wrz, wrw;			//World rotation
			public double wsx, wsy, wsz;				//World scale
			public double lpx, lpy, lpz;				//Local position
			public double lrx, lry, lrz, lrw;			//Local rotation

			public bool xEnabled;
			public bool yEnabled;
			public bool zEnabled;
			public int xIndex = -1;
			public int yIndex = -1;
			public int zIndex = -1;
			public double xValue;
			public double yValue;
			public double zValue;
		
			public bool[] objectiveImpacts;				//Boolean values to represent which objective indices in the whole kinematic tree are affected

			//Setup for the node
			public Node(Model model, Node parent, BioSegment segment)
			{
				_model = model;
				this.parent = parent;
				this.parent?.AddChild(this);
				transform = segment.transform;
				joint = segment.joint;

				List<Transform> reverseChain = new() { transform };
				Node p = parent;
				while (p != null)
				{
					reverseChain.Add(p.transform);
					p = p.parent;
				}
				reverseChain.Reverse();
				chain = reverseChain.ToArray();
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
				if (joint == null)
				{
					Vector3 lp = transform.localPosition;
					Quaternion lr = transform.localRotation;
					lpx = lp.x;
					lpy = lp.y;
					lpz = lp.z;
					lrx = lr.x;
					lry = lr.y;
					lrz = lr.z;
					lrw = lr.w;
				}
				else
				{
					xValue = joint.x.GetTargetValue(true);
					yValue = joint.y.GetTargetValue(true);
					zValue = joint.z.GetTargetValue(true);
					joint.ComputeLocalTransformation(xValue, yValue, zValue, out lpx, out lpy, out lpz, out lrx, out lry, out lrz, out lrw);
				}
				Vector3 ws = transform.lossyScale;
				wsx = ws.x;
				wsy = ws.y;
				wsz = ws.z;

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

				if (xEnabled)
				{
					xValue = configuration[xIndex];
					updateLocal = true;
				}
				
				if (yEnabled)
				{
					yValue = configuration[yIndex];
					updateLocal = true;
				}
				
				if (zEnabled)
				{
					zValue = configuration[zIndex];
					updateLocal = true;
				}
				
				//Only update local transformation if a joint value has changed
				if (updateLocal)
				{
					joint.ComputeLocalTransformation(xValue, yValue, zValue, out lpx, out lpy, out lpz, out lrx, out lry, out lrz, out lrw);
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

				Node node = _model._objectivePointer.node;
				if (objectiveImpacts[0])
				{
					joint.ComputeLocalTransformation(
						xEnabled ? configuration[xIndex] : xValue,
						yEnabled ? configuration[yIndex] : yValue, 
						zEnabled ? configuration[zIndex] : zValue, 
						out double lpX, out double lpY, out double lpZ, out double lrX, out double lrY, out double lrZ, out double lrW
					);
					double localRx, localRy, localRz, localRw, localX, localY, localZ;
					if (parent == null)
					{
						px[0] = _model._opx;
						py[0] = _model._opy;
						pz[0] = _model._opz;
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
						px[0] = parent.wpx;
						py[0] = parent.wpy;
						pz[0] = parent.wpz;
						localRx = parent.wrx;
						localRy = parent.wry;
						localRz = parent.wrz;
						localRw = parent.wrw;
						localX = parent.wsx*lpX;
						localY = parent.wsy*lpY;
						localZ = parent.wsz*lpZ;
					}
					double qx = localRx * lrW + localRy * lrZ - localRz * lrY + localRw * lrX;
					double qy = -localRx * lrZ + localRy * lrW + localRz * lrX + localRw * lrY;
					double qz = localRx * lrY - localRy * lrX + localRz * lrW + localRw * lrZ;
					double qw = -localRx * lrX - localRy * lrY - localRz * lrZ + localRw * lrW;
					double dot = wrx*wrx + wry*wry + wrz*wrz + wrw*wrw;
					double x = qx / dot; double y = qy / dot; double z = qz / dot; double w = qw / dot;
					qx = x * wrw + y * -wrz - z * -wry + w * -wrx;
					qy = -x * -wrz + y * wrw + z * -wrx + w * -wry;
					qz = x * -wry - y * -wrx + z * wrw + w * -wrz;
					qw = -x * -wrx - y * -wry - z * -wrz + w * wrw;
					px[0] +=
							+ 2.0 * ((0.5 - localRy * localRy - localRz * localRz) * localX + (localRx * localRy - localRw * localRz) * localY + (localRx * localRz + localRw * localRy) * localZ)
							+ 2.0 * ((0.5 - qy * qy - qz * qz) * (node.wpx-wpx) + (qx * qy - qw * qz) * (node.wpy-wpy) + (qx * qz + qw * qy) * (node.wpz-wpz));
					py[0] += 
							+ 2.0 * ((localRx * localRy + localRw * localRz) * localX + (0.5 - localRx * localRx - localRz * localRz) * localY + (localRy * localRz - localRw * localRx) * localZ)
							+ 2.0 * ((qx * qy + qw * qz) * (node.wpx-wpx) + (0.5 - qx * qx - qz * qz) * (node.wpy-wpy) + (qy * qz - qw * qx) * (node.wpz-wpz));
					pz[0] += 
							+ 2.0 * ((localRx * localRz - localRw * localRy) * localX + (localRy * localRz + localRw * localRx) * localY + (0.5 - (localRx * localRx + localRy * localRy)) * localZ)
							+ 2.0 * ((qx * qz - qw * qy) * (node.wpx-wpx) + (qy * qz + qw * qx) * (node.wpy-wpy) + (0.5 - qx * qx - qy * qy) * (node.wpz-wpz));
					rx[0] = qx * node.wrw + qy * node.wrz - qz * node.wry + qw * node.wrx;
					ry[0] = -qx * node.wrz + qy * node.wrw + qz * node.wrx + qw * node.wry;
					rz[0] = qx * node.wry - qy * node.wrx + qz * node.wrw + qw * node.wrz;
					rw[0] = -qx * node.wrx - qy * node.wry - qz * node.wrz + qw * node.wrw;
					_model._simulatedLosses[0] = _model._objectivePointer.objective.ComputeLoss(px[0], py[0], pz[0], rx[0], ry[0], rz[0], rw[0]);
				}
				else
				{
					px[0] = node.wpx;
					py[0] = node.wpy;
					pz[0] = node.wpz;
					rx[0] = node.wrx;
					ry[0] = node.wry;
					rz[0] = node.wrz;
					rw[0] = node.wrw;
					_model._simulatedLosses[0] = _model._losses[0];
				}
			}

			//Computes the world transformation using the current joint variable configuration
			private void ComputeWorldTransformation()
			{
				double rx, ry, rz, rw, x, y, z;
				if (parent == null)
				{
					wpx = _model._opx;
					wpy = _model._opy;
					wpz = _model._opz;
					rx = _model._orx;
					ry = _model._ory;
					rz = _model._orz;
					rw = _model._orw;
					x = _model._osx*lpx;
					y = _model._osy*lpy;
					z = _model._osz*lpz;
				}
				else
				{
					wpx = parent.wpx;
					wpy = parent.wpy;
					wpz = parent.wpz;
					rx = parent.wrx;
					ry = parent.wry;
					rz = parent.wrz;
					rw = parent.wrw;
					x = parent.wsx*lpx;
					y = parent.wsy*lpy;
					z = parent.wsz*lpz;
				}
				wpx += 2.0 * ((0.5 - ry * ry - rz * rz) * x + (rx * ry - rw * rz) * y + (rx * rz + rw * ry) * z);
				wpy += 2.0 * ((rx * ry + rw * rz) * x + (0.5 - rx * rx - rz * rz) * y + (ry * rz - rw * rx) * z);
				wpz += 2.0 * ((rx * rz - rw * ry) * x + (ry * rz + rw * rx) * y + (0.5 - rx * rx - ry * ry) * z);
				wrx = rx * lrw + ry * lrz - rz * lry + rw * lrx;
				wry = -rx * lrz + ry * lrw + rz * lrx + rw * lry;
				wrz = rx * lry - ry * lrx + rz * lrw + rw * lrz;
				wrw = -rx * lrx - ry * lry - rz * lrz + rw * lrw;
			}
		}

		//Data class to store pointers to the objectives
		public class ObjectivePtr
		{
			public readonly BioObjective objective;
			public readonly Node node;

			public ObjectivePtr(BioObjective objective, Node node)
			{
				this.objective = objective;
				this.node = node;
			}
		}

		//Data class to store pointers to the joint motions
		public class MotionPtr
		{
			public readonly BioJoint.Motion motion;
			public readonly Node node;
			public readonly int index;
			
			public MotionPtr(BioJoint.Motion motion, Node node, int index)
			{
				this.motion = motion;
				this.node = node;
				this.index = index;
			}
		}
	}
}