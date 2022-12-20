using System;
using System.Collections.Generic;
using BioIK.Setup;
using UnityEngine;

namespace BioIK.Helpers
{
	public class Model
	{
		private readonly BioRobot _bioRobot;

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
		private double _losses;

		//Simulated Configuration
		private double _px;
		private double _py;
		private double _pz;
		private double _rx;
		private double _ry;
		private double _rz;
		private double _rw;
		private double _simulatedLosses;

		//Degree of Freedom
		private readonly int _doF;

		public Model(BioRobot bioRobot)
		{
			_bioRobot = bioRobot;

			//Set Root
			_root = _bioRobot.root;

			//Create Root
			AddNode(_root);
			
			//Build Model
			BioObjective objective = FindObjective(_root);
			if (objective != null)
			{
				List<BioSegment> chain = _bioRobot.GetChain(objective.segment);
				for (int j = 1; j < chain.Count; j++)
				{
					AddNode(chain[j]);
				}
			}

			//Assign DoF
			_doF = motionPointers.Length;
			_configuration = new double[motionPointers.Length];
			_gradient = new double[motionPointers.Length];

			Refresh();
		}

		public int GetDoF()
		{
			return _doF;
		}

		public BioRobot GetCharacter()
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
			_px = model._px;
			_py = model._py;
			_pz = model._pz;
			_rx = model._rx;
			_ry = model._ry;
			_rz = model._rz;
			_rw = model._rw;
			_losses = model._losses;
			_simulatedLosses = model._simulatedLosses;
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
			_losses = _objectivePointer.objective.ComputeLoss(node.wpx, node.wpy, node.wpz, node.wrx, node.wry, node.wrz, node.wrw);
			return Math.Sqrt(_losses);
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
				_gradient[j] = (Math.Sqrt(_simulatedLosses) - oldLoss) / resolution;
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
				if (node.joint.GetDoF() == 0)
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
			if (objective != null)
			{
				_objectivePointer = new(objective, node);
			}

			Array.Resize(ref _nodes, _nodes.Length + 1);
			_nodes[^1] = node;
		}

		private static BioObjective FindObjective(BioSegment segment)
		{
			while (segment != null)
			{
				if (segment.objective != null)
				{
					return segment.objective;
				}

				segment = segment.child;
			}

			return null;
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
				Node node = _model._objectivePointer.node;
				joint.ComputeLocalTransformation(
					xEnabled ? configuration[xIndex] : xValue,
					yEnabled ? configuration[yIndex] : yValue, 
					zEnabled ? configuration[zIndex] : zValue, 
					out double lpX, out double lpY, out double lpZ, out double lrX, out double lrY, out double lrZ, out double lrW
				);
				double localRx, localRy, localRz, localRw, localX, localY, localZ;
				double px;
				double py;
				double pz;
				if (parent == null)
				{
					px = _model._opx;
					py = _model._opy;
					pz = _model._opz;
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
					px = parent.wpx;
					py = parent.wpy;
					pz = parent.wpz;
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
				px +=
						+ 2.0 * ((0.5 - localRy * localRy - localRz * localRz) * localX + (localRx * localRy - localRw * localRz) * localY + (localRx * localRz + localRw * localRy) * localZ)
						+ 2.0 * ((0.5 - qy * qy - qz * qz) * (node.wpx-wpx) + (qx * qy - qw * qz) * (node.wpy-wpy) + (qx * qz + qw * qy) * (node.wpz-wpz));
				py += 
						+ 2.0 * ((localRx * localRy + localRw * localRz) * localX + (0.5 - localRx * localRx - localRz * localRz) * localY + (localRy * localRz - localRw * localRx) * localZ)
						+ 2.0 * ((qx * qy + qw * qz) * (node.wpx-wpx) + (0.5 - qx * qx - qz * qz) * (node.wpy-wpy) + (qy * qz - qw * qx) * (node.wpz-wpz));
				pz += 
						+ 2.0 * ((localRx * localRz - localRw * localRy) * localX + (localRy * localRz + localRw * localRx) * localY + (0.5 - (localRx * localRx + localRy * localRy)) * localZ)
						+ 2.0 * ((qx * qz - qw * qy) * (node.wpx-wpx) + (qy * qz + qw * qx) * (node.wpy-wpy) + (0.5 - qx * qx - qy * qy) * (node.wpz-wpz));
				double rx = qx * node.wrw + qy * node.wrz - qz * node.wry + qw * node.wrx;
				double ry = -qx * node.wrz + qy * node.wrw + qz * node.wrx + qw * node.wry;
				double rz = qx * node.wry - qy * node.wrx + qz * node.wrw + qw * node.wrz;
				double rw = -qx * node.wrx - qy * node.wry - qz * node.wrz + qw * node.wrw;
				_model._simulatedLosses = _model._objectivePointer.objective.ComputeLoss(px, py, pz, rx, ry, rz, rw);
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