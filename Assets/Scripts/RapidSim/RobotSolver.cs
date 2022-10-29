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

        public NeuralNetwork Net { get; private set; }
        
        private RobotController _robotController;

        public int NetworkSteps => Net.step;

        private void Start()
        {
            _robotController = GetComponent<RobotController>();

            if (data != null && data.HasModel)
            {
                Net = data.Load();
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

            Net = new(layers);
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

        private List<float> Solve(Vector3 position, Quaternion rotation)
        {
            float[] joints = JointsScaled(Net.Forward(PrepareInputs(NetScaled(_robotController.GetJoints().ToArray()), position, rotation)));

            for (int i = 0; i < joints.Length; i++)
            {
                joints[i] = Mathf.Clamp(joints[i], _robotController.Limits[i].Lower, _robotController.Limits[i].Upper);
            }
        
            // TODO: Finalize movement with Hybrid IK.

            return joints.ToList();
        }

        public float[] PrepareInputs(float[] joints, Vector3 position, Quaternion rotation)
        {
            float[] inputs = new float[7 + joints.Length];
            position = RelativePosition(position) / _robotController.ChainLength;
            inputs[0] = position.x;
            inputs[1] = position.y;
            inputs[2] = position.z;
            rotation = RelativeRotation(rotation);
            inputs[3] = rotation.x;
            inputs[4] = rotation.y;
            inputs[5] = rotation.z;
            inputs[6] = rotation.w;
            for (int i = 0; i < joints.Length; i++)
            {
                inputs[i + 7] = joints[i];
            }
            return inputs;
        }
    
        public float[] NetScaled(float[] joints)
        {
            for (int i = 0; i < joints.Length; i++)
            {
                joints[i] = (joints[i] - _robotController.Limits[i].Lower) / (_robotController.Limits[i].Upper - _robotController.Limits[i].Lower);
            }

            return joints;
        }
    
        public float[] JointsScaled(float[] joints)
        {
            for (int i = 0; i < joints.Length; i++)
            {
                joints[i] = Mathf.Clamp(joints[i] * (_robotController.Limits[i].Upper - _robotController.Limits[i].Lower) + _robotController.Limits[i].Lower, _robotController.Limits[i].Lower, _robotController.Limits[i].Upper);
            }

            return joints;
        }

        public Vector3 RelativePosition(Vector3 position) => _robotController.Root.transform.InverseTransformPoint(position);
    
        public Quaternion RelativeRotation(Quaternion rotation) => Quaternion.Inverse(_robotController.Root.transform.rotation) * rotation;
    }
}