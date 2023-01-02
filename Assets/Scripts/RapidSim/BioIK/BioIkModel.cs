using System;
using Unity.Mathematics;
using UnityEngine;

namespace RapidSim.BioIK
{
	public class BioIkModel
	{
		private readonly Robot _robot;

		// Reference to root
		private readonly BioIkJoint _root;

		// Offset to world
		private double _opx, _opy, _opz;
		private double _orx, _ory, _orz, _orw;
		
		// Linked list of nodes in the model
		private Node[] _nodes = Array.Empty<Node>();

		//Global pointers to the IK setup
		public MotionPtr[] motionPointers = Array.Empty<MotionPtr>();

		//Assigned Configuration
		private readonly double[] _configuration;
		private readonly double[] _gradient;
		private double _loss;

		//Simulated Configuration
		private double _px;
		private double _py;
		private double _pz;
		private double _rx;
		private double _ry;
		private double _rz;
		private double _rw;
		private double _simulatedLoss;

		//Degree of Freedom
		private readonly int _doF;
		
		private double _tpx, _tpy, _tpz;
		private double _trx, _try, _trz, _trw;
		private double _rescaling;

		public BioIkModel(Robot robot, double rescaling)
		{
			_robot = robot;

			//Set Root
			_root = _robot.RootJoint;

			AddNode(_root, null);
			BioIkJoint current = _root.child;
			while (current != null)
			{
				AddNode(current, _nodes[^1]);
				current = current.child;
			}
			
			_rescaling = rescaling;

			//Assign DoF
			_doF = motionPointers.Length;
			_configuration = new double[motionPointers.Length];
			_gradient = new double[motionPointers.Length];

			Refresh();
		}

		public void SetTargetPosition(Vector3 position)
		{
			_tpx = position.x;
			_tpy = position.y;
			_tpz = position.z;
		}
		
		public void SetTargetRotation(Quaternion rotation)
		{
			_trx = rotation.x;
			_try = rotation.y;
			_trz = rotation.z;
			_trw = rotation.w;
		}

		public int GetDoF()
		{
			return _doF;
		}

		public Robot GetBioRobot()
		{
			return _robot;
		}

		public void Refresh()
		{
			//Updates configuration
			for (int i = 0; i < _configuration.Length; i++)
			{
				_configuration[i] = motionPointers[i].motion.GetTargetValue();
			}

			//Update offset from world to root
			if (_root.transform.root == _robot.transform)
			{
				_opx = _opy = _opz = _orx = _ory = _orz = 0.0;
				_orw = 1.0;
			}
			else
			{
				Transform parent = _root.transform.parent;
				Vector3 p = parent.position;
				Quaternion r = parent.rotation;
				_opx = p.x; _opy = p.y; _opz = p.z;
				_orx = r.x; _ory = r.y; _orz = r.z; _orw = r.w;
			}

			//Updates the nodes
			_nodes[0].Refresh();
		}

		public void CopyFrom(BioIkModel bioIkModel)
		{
			_opx = bioIkModel._opx;
			_opy = bioIkModel._opy;
			_opz = bioIkModel._opz;
			_orx = bioIkModel._orx;
			_ory = bioIkModel._ory;
			_orz = bioIkModel._orz;
			_orw = bioIkModel._orw;
			for (int i = 0; i < _doF; i++)
			{
				_configuration[i] = bioIkModel._configuration[i];
				_gradient[i] = bioIkModel._gradient[i];
			}
			_px = bioIkModel._px;
			_py = bioIkModel._py;
			_pz = bioIkModel._pz;
			_rx = bioIkModel._rx;
			_ry = bioIkModel._ry;
			_rz = bioIkModel._rz;
			_rw = bioIkModel._rw;
			_loss = bioIkModel._loss;
			_simulatedLoss = bioIkModel._simulatedLoss;
			for (int i = 0; i < _nodes.Length; i++)
			{
				_nodes[i].wpx = bioIkModel._nodes[i].wpx;
				_nodes[i].wpy = bioIkModel._nodes[i].wpy;
				_nodes[i].wpz = bioIkModel._nodes[i].wpz;
				_nodes[i].wrx = bioIkModel._nodes[i].wrx;
				_nodes[i].wry = bioIkModel._nodes[i].wry;
				_nodes[i].wrz = bioIkModel._nodes[i].wrz;
				_nodes[i].wrw = bioIkModel._nodes[i].wrw;
				_nodes[i].wsx = bioIkModel._nodes[i].wsx;
				_nodes[i].wsy = bioIkModel._nodes[i].wsy;
				_nodes[i].wsz = bioIkModel._nodes[i].wsz;

				_nodes[i].lpx = bioIkModel._nodes[i].lpx;
				_nodes[i].lpy = bioIkModel._nodes[i].lpy;
				_nodes[i].lpz = bioIkModel._nodes[i].lpz;
				_nodes[i].lrx = bioIkModel._nodes[i].lrx;
				_nodes[i].lry = bioIkModel._nodes[i].lry;
				_nodes[i].lrz = bioIkModel._nodes[i].lrz;
				_nodes[i].lrw = bioIkModel._nodes[i].lrw;
				
				_nodes[i].xValue = bioIkModel._nodes[i].xValue;
				_nodes[i].yValue = bioIkModel._nodes[i].yValue;
				_nodes[i].zValue = bioIkModel._nodes[i].zValue;
			}
			_rescaling = bioIkModel._rescaling;
			_tpx = bioIkModel._tpx;
			_tpy = bioIkModel._tpy;
			_tpz = bioIkModel._tpz;
			_trx = bioIkModel._trx;
			_try = bioIkModel._try;
			_trz = bioIkModel._trz;
			_trw = bioIkModel._trw;
			_rescaling = bioIkModel._rescaling;
		}

		//Computes the loss as the root mean error squared over all objectives
		public double ComputeLoss(double[] configuration)
		{
			ForwardKinematics(configuration);
			Node node = motionPointers[^1].node;
			_loss = ComputeLoss(node.wpx, node.wpy, node.wpz, node.wrx, node.wry, node.wrz, node.wrw);
			return Math.Sqrt(_loss);
		}

		private double ComputeLoss(double apx, double apy, double apz, double arx, double ary, double arz, double arw)
		{
			double pos = _rescaling * ((_tpx - apx) * (_tpx - apx) + (_tpy - apy) * (_tpy - apy) + (_tpz - apz) * (_tpz - apz));
			
			double d = arx * _trx + ary * _try + arz * _trz + arw * _trw;
			switch (d)
			{
				case < 0.0:
				{
					d = -d;
					if(d > 1.0) {
						d = 1.0;
					}

					break;
				}
				case > 1.0:
					d = 1.0;
					break;
			}
			double rot = 2.0 * math.acos(d);
			rot *= rot;
			
			return pos + rot;
		}

		public bool CheckConvergence(double[] configuration, double repeatability)
		{
			ForwardKinematics(configuration);
			Node node = motionPointers[^1].node;
			return ComputeLoss(node.wpx, node.wpy, node.wpz, node.wrx, node.wry, node.wrz, node.wrw) <= repeatability;
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
				_gradient[j] = (Math.Sqrt(_simulatedLoss) - oldLoss) / resolution;
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
		private void AddNode(BioIkJoint ikSegment, Node parent)
		{
			Node node = new(this, parent, ikSegment);

			if (node.bioIkJoint != null)
			{
				if (node.bioIkJoint.GetDoF() == 0)
				{
					node.bioIkJoint = null;
				}
				else
				{
					if (node.bioIkJoint.x.enabled)
					{
						MotionPtr motionPtr = new(node.bioIkJoint.x, node, motionPointers.Length);
						Array.Resize(ref motionPointers, motionPointers.Length + 1);
						motionPointers[^1] = motionPtr;
						node.xEnabled = true;
						node.xIndex = motionPtr.index;
					}
					if (node.bioIkJoint.y.enabled)
					{
						MotionPtr motionPtr = new(node.bioIkJoint.y, node, motionPointers.Length);
						Array.Resize(ref motionPointers, motionPointers.Length + 1);
						motionPointers[^1] = motionPtr;
						node.yEnabled = true;
						node.yIndex = motionPtr.index;
					}
					if (node.bioIkJoint.z.enabled)
					{
						MotionPtr motionPtr = new(node.bioIkJoint.z, node, motionPointers.Length);
						Array.Resize(ref motionPointers, motionPointers.Length + 1);
						motionPointers[^1] = motionPtr;
						node.zEnabled = true;
						node.zIndex = motionPtr.index;
					}
				}
			}

			Array.Resize(ref _nodes, _nodes.Length + 1);
			_nodes[^1] = node;
		}

		// Subclass representing the single nodes for the data structure.
		// Values are stored using primitive data types for faster access and efficient computation.
		public class Node
		{
			private readonly BioIkModel _bioIkModel;							//Reference to the kinematic model
			private readonly Node _parent;							//Reference to the parent of this node
			private Node _child;
			private readonly Transform _transform;					//Reference to the transform
			public BioIkJoint bioIkJoint;									//Reference to the joint

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
			public Node(BioIkModel bioIkModel, Node parent, BioIkJoint bioIkJoint)
			{
				_bioIkModel = bioIkModel;
				_parent = parent;
				if (_parent != null)
				{
					_parent._child = this;
				}

				_transform = bioIkJoint.transform;
				this.bioIkJoint = bioIkJoint;
			}

			//Recursively refreshes the current transform data
			public void Refresh()
			{
				xValue = bioIkJoint.x.GetTargetValue();
				yValue = bioIkJoint.y.GetTargetValue();
				zValue = bioIkJoint.z.GetTargetValue();
				bioIkJoint.ComputeLocalTransformation(xValue, yValue, zValue, out lpx, out lpy, out lpz, out lrx, out lry, out lrz, out lrw);
				
				Vector3 ws = _transform.lossyScale;
				wsx = ws.x;
				wsy = ws.y;
				wsz = ws.z;

				//World
				ComputeWorldTransformation();

				//Feed Forward
				_child?.Refresh();
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
					bioIkJoint.ComputeLocalTransformation(xValue, yValue, zValue, out lpx, out lpy, out lpz, out lrx, out lry, out lrz, out lrw);
					updateWorld = true;
				}

				//Only update world transformation if local transformation (in this or parent node) has changed
				if (updateWorld)
				{
					ComputeWorldTransformation();
				}

				//Feed forward the joint variable configuration
				_child?.FeedForwardConfiguration(configuration, updateWorld);
			}

			//Simulates a single transform modification while leaving the whole data structure unchanged
			//Returns the resulting Cartesian posture transformations in the out values
			public void SimulateModification(double[] configuration)
			{
				Node node = _bioIkModel.motionPointers[^1].node;
				bioIkJoint.ComputeLocalTransformation(
					xEnabled ? configuration[xIndex] : xValue,
					yEnabled ? configuration[yIndex] : yValue, 
					zEnabled ? configuration[zIndex] : zValue, 
					out double lpX, out double lpY, out double lpZ, out double lrX, out double lrY, out double lrZ, out double lrW
				);
				double localRx, localRy, localRz, localRw, localX, localY, localZ;
				double px;
				double py;
				double pz;
				if (_parent == null)
				{
					px = _bioIkModel._opx;
					py = _bioIkModel._opy;
					pz = _bioIkModel._opz;
					localRx = _bioIkModel._orx;
					localRy = _bioIkModel._ory;
					localRz = _bioIkModel._orz;
					localRw = _bioIkModel._orw;
					localX = lpX;
					localY = lpY;
					localZ = lpZ;
				}
				else
				{
					px = _parent.wpx;
					py = _parent.wpy;
					pz = _parent.wpz;
					localRx = _parent.wrx;
					localRy = _parent.wry;
					localRz = _parent.wrz;
					localRw = _parent.wrw;
					localX = _parent.wsx * lpX;
					localY = _parent.wsy * lpY;
					localZ = _parent.wsz * lpZ;
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
				_bioIkModel._simulatedLoss = _bioIkModel.ComputeLoss(px, py, pz, rx, ry, rz, rw);
			}

			//Computes the world transformation using the current joint variable configuration
			private void ComputeWorldTransformation()
			{
				double rx, ry, rz, rw, x, y, z;
				if (_parent == null)
				{
					wpx = _bioIkModel._opx;
					wpy = _bioIkModel._opy;
					wpz = _bioIkModel._opz;
					rx = _bioIkModel._orx;
					ry = _bioIkModel._ory;
					rz = _bioIkModel._orz;
					rw = _bioIkModel._orw;
					x = lpx;
					y = lpy;
					z = lpz;
				}
				else
				{
					wpx = _parent.wpx;
					wpy = _parent.wpy;
					wpz = _parent.wpz;
					rx = _parent.wrx;
					ry = _parent.wry;
					rz = _parent.wrz;
					rw = _parent.wrw;
					x = _parent.wsx * lpx;
					y = _parent.wsy * lpy;
					z = _parent.wsz * lpz;
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

		//Data class to store pointers to the joint motions
		public struct MotionPtr
		{
			public readonly BioIkJoint.Motion motion;
			public readonly Node node;
			public readonly int index;
			
			public MotionPtr(BioIkJoint.Motion motion, Node node, int index)
			{
				this.motion = motion;
				this.node = node;
				this.index = index;
			}
		}
	}
}