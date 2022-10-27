using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace RapidSim
{
    [DisallowMultipleComponent]
    public class RobotController : MonoBehaviour
    {
        public ArticulationBody Root => Joints[0].Joint;

        public Transform LastJoint => Joints[^1].transform;

        private List<float> _home;

        private List<float> _zeros;

        private List<float> _targets;

        private bool _move;
        
        private float[] _maxSpeeds;

        private float[] _currentSpeeds;

        public JointLimit[] Limits { get; private set; }

        public float ChainLength { get; private set; }

        public RobotJoint[] Joints { get; private set; }

        public void Start()
        {
            RobotJoint root = GetComponent<RobotJoint>();
        
            RobotJoint[] children = GetComponentsInChildren<RobotJoint>();

            if (root == null)
            {
                if (children.Length == 0)
                {
                    Debug.LogError($"No articulation bodies attached to {name}.");
                    Destroy(this);
                    return;
                }

                Joints = children;
            }
            else if (children.Length > 0)
            {
                Joints = new RobotJoint[children.Length + 1];
                ChainLength = Vector3.Distance(Root.transform.position, children[0].transform.position);
                Joints[0] = root;
                for (int i = 1; i < Joints.Length - 1; i++)
                {
                    Joints[i] = children[i - 1];
                }
            }
            else
            {
                Joints = new RobotJoint[1];
                Joints[0] = root;
            }

            Joints = Joints.OrderBy(j => j.Joint.index).ToArray();

            List<JointLimit> limits = new();
            for (int i = 0; i < Joints.Length; i++)
            {
                limits.AddRange(Joints[i].Limits());
            }

            Limits = limits.ToArray();
            _home = GetJoints();
            GetMaxSpeeds();

            if (_home.Count != Limits.Length)
            {
                Debug.LogError($"Ensure all joints on {name} have limits defined.");
            }
        
            _zeros = new();
            for (int i = 0; i < _home.Count; i++)
            {
                _zeros.Add(0);
            }
            
            _currentSpeeds = new float[_maxSpeeds.Length];
            for (int i = 0; i < _currentSpeeds.Length; i++)
            {
                _currentSpeeds[i] = _maxSpeeds[i];
            }

            if (_home.Count != _maxSpeeds.Length)
            {
                Debug.LogError($"{name} has {_home.Count} degrees of freedom but {_maxSpeeds.Length} speeds defined.");
            }
        }

        public void Move(List<float> degrees)
        {
            MoveRadians(DegreesToRadians(degrees));
        }

        public void MoveRadians(List<float> radians)
        {
            _targets = radians;

            List<float> angles = GetJoints();
            float time = 0;
            for (int i = 0; i < angles.Count; i++)
            {
                angles[i] = Mathf.Abs(angles[i] - _targets[i]);
                if (angles[i] / _maxSpeeds[i] > time)
                {
                    time = angles[i] / _maxSpeeds[i];
                }
            }

            if (time > 0)
            {
                for (int i = 0; i < _currentSpeeds.Length; i++)
                {
                    _currentSpeeds[i] = angles[i] / time;
                }
            }

            _move = true;
        }

        public void Snap(List<float> degrees)
        {
            SnapRadians(DegreesToRadians(degrees));
        }

        public void SnapRadians(List<float> radians)
        {
            _move = false;
            Stop(radians);
        }

        public void MoveHome()
        {
            MoveRadians(_home);
        }

        public void SnapHome()
        {
            SnapRadians(_home);
        }

        public List<float> GetJoints()
        {
            List<float> angles = new();
            Root.GetJointPositions(angles);
            return angles;
        }

        public void GetMaxSpeeds()
        {
            List<float> speeds = new();
            
            for (int i = 0; i < Joints.Length; i++)
            {
                if (!Joints[i].HasMotion)
                {
                    continue;
                }

                if (Joints[i].XMotion)
                {
                    speeds.Add(Joints[i].SpeedX);
                }

                if (Joints[i].YMotion)
                {
                    speeds.Add(Joints[i].SpeedY);
                }

                if (Joints[i].ZMotion)
                {
                    speeds.Add(Joints[i].SpeedZ);
                }
            }

            _maxSpeeds = speeds.ToArray();
        }

        private void FixedUpdate()
        {
            if (!_move)
            {
                return;
            }

            _move = false;

            List<float> delta = GetJoints();
            for (int i = 0; i < delta.Count; i++)
            {
                if (delta[i] >= _targets[i])
                {
                    delta[i] -= _currentSpeeds[i] * Time.fixedDeltaTime;
                    if (delta[i] <= _targets[i])
                    {
                        delta[i] = _targets[i];
                    }
                    else
                    {
                        _move = true;
                    }
                }
                else
                {
                    delta[i] += _currentSpeeds[i] * Time.fixedDeltaTime;
                    if (delta[i] >= _targets[i])
                    {
                        delta[i] = _targets[i];
                    }
                    else
                    {
                        _move = true;
                    }
                }
            }

            if (_move)
            {
                Root.SetDriveTargets(delta);
            }
            else
            {
                Stop(delta);
            }
        }

        private void Stop(IEnumerable<float> radians)
        {
            List<float> list = radians.ToList();
            Root.SetDriveTargets(list);
            Root.SetJointVelocities(_zeros);
            Root.SetJointAccelerations(_zeros);
            Root.SetJointForces(_zeros);
            Root.SetJointPositions(list);
        }

        private static List<float> DegreesToRadians(List<float> degrees)
        {
            for (int i = 0; i < degrees.Count; i++)
            {
                degrees[i] *= Mathf.Deg2Rad;
            }

            return degrees;
        }
    }
}