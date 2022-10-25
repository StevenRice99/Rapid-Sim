using System.Collections.Generic;
using System.Linq;
using RapidSim.Networks;
using UnityEngine;

namespace RapidSim
{
    [DisallowMultipleComponent]
    [RequireComponent(typeof(RobotController))]
    public class RobotSolver : MonoBehaviour
    {
        [SerializeField]
        private NeuralNetworkData data;
        
        private RobotController _robotController;

        private NeuralNetwork _net;

        private void Start()
        {
            _robotController = GetComponent<RobotController>();

            if (data != null && data.HasModel)
            {
                _net = data.Load();
                return;
            }

            int s = _robotController.GetJoints().Count;
            int[] layers = new int[s + 2];
            layers[^1] = s;
            s += 7;
            layers[0] = s;
            s *= 2;
            for (int i = 1; i < layers.Length - 1; i++)
            {
                layers[i] = s;
            }

            _net = new(layers);
            Debug.Log(_net);
        }
    
        public void Move(GameObject target)
        {
            Move(target.transform);
        }

        public void Move(Component target)
        {
            Move(target.transform);
        }

        public void Move(Transform target)
        {
            Move(target.position, target.rotation);
        }

        public void Move(Vector3 position)
        {
            Move(position, _robotController.LastJoint.rotation);
        }

        public void Move(Quaternion rotation)
        {
            Move(_robotController.LastJoint.position, rotation);
        }

        public void Move(Vector3 position, Quaternion rotation)
        {
            _robotController.MoveRadians(Solve(position, rotation));
        }
    
        public void Snap(GameObject target)
        {
            Snap(target.transform);
        }

        public void Snap(Component target)
        {
            Snap(target.transform);
        }

        public void Snap(Transform target)
        {
            Snap(target.position, target.rotation);
        }

        public void Snap(Vector3 position)
        {
            Snap(position, _robotController.LastJoint.rotation);
        }

        public void Snap(Quaternion rotation)
        {
            Snap(_robotController.LastJoint.position, rotation);
        }
    
        public void Snap(Vector3 position, Quaternion rotation)
        {
            _robotController.SnapRadians(Solve(position, rotation));
        }

        public void Train(Vector3 position, Quaternion rotation, List<float> joints, float[] expected)
        {
            _net.Train(PrepareInputs(NetScaled(joints), position, rotation), NetScaled(expected.ToList()).ToArray());
        }

        private List<float> Solve(Vector3 position, Quaternion rotation)
        {
            float[] joints = RadianScaled(_net.Forward(PrepareInputs(NetScaled(_robotController.GetJoints()), position, rotation)));
        
            // TODO: Finalize movement with Hybrid IK.

            return joints.ToList();
        }

        private List<float> Solve(List<float> inputs)
        {
            return _net.Forward(inputs.ToArray()).ToList();
        }

        public float[] PrepareInputs(List<float> joints, Vector3 position, Quaternion rotation)
        {
            float[] inputs = new float[7 + joints.Count];
            position = RelativePosition(position);
            inputs[0] = position.x;
            inputs[1] = position.y;
            inputs[2] = position.z;
            rotation = RelativeRotation(rotation);
            inputs[3] = rotation.x;
            inputs[4] = rotation.y;
            inputs[5] = rotation.z;
            inputs[6] = rotation.w;
            for (int i = 0; i < joints.Count; i++)
            {
                inputs[i + 7] = joints[i];
            }
            return inputs;
        }
    
        private List<float> NetScaled(List<float> joints)
        {
            for (int i = 0; i < joints.Count; i++)
            {
                joints[i] = (joints[i] - _robotController.LowerLimits[i]) / (_robotController.UpperLimits[i] - _robotController.LowerLimits[i]);
            }

            return joints;
        }
    
        private float[] RadianScaled(float[] joints)
        {
            for (int i = 0; i < joints.Length; i++)
            {
                joints[i] = Mathf.Clamp(joints[i] * (_robotController.UpperLimits[i] - _robotController.LowerLimits[i]) + _robotController.LowerLimits[i], _robotController.LowerLimits[i], _robotController.UpperLimits[i]);
            }

            return joints;
        }

        public Vector3 RelativePosition(Vector3 position) => _robotController.Root.transform.InverseTransformPoint(position);
    
        public Quaternion RelativeRotation(Quaternion rotation) => Quaternion.Inverse(_robotController.Root.transform.rotation) * rotation;
    }
}