using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace RapidSim
{
    [DisallowMultipleComponent]
    public class RobotController : MonoBehaviour
    {
        [SerializeField]
        private float[] maxSpeeds;

        public ArticulationBody Root { get; private set; }

        public Transform LastJoint { get; private set; }

        private List<float> _home;

        private List<float> _zeros;

        private List<float> _targets;

        private bool _move;

        private float[] _currentSpeeds;

        public float[] LowerLimits { get; private set; }

        public float[] UpperLimits { get; private set; }

        public float ChainLength { get; private set; }

        public void Start()
        {
            Root = GetComponent<ArticulationBody>();
        
            ArticulationBody[] children = GetComponentsInChildren<ArticulationBody>();

            if (Root == null)
            {
                if (children.Length == 0)
                {
                    Debug.LogError($"No articulation bodies attached to {name}.");
                    Destroy(this);
                    return;
                }

                Root = children[0];
            }
            else if (children.Length > 0)
            {
                ChainLength = Vector3.Distance(Root.transform.position, children[0].transform.position);
            }

            _home = GetJoints();
            LowerLimits = new float[_home.Count];
            UpperLimits = new float[_home.Count];

            ArticulationBody last = Root;

            int dofIndex = 0;
        
            for (int i = 0; i < children.Length; i++)
            {
                if (children[i].index > last.index)
                {
                    last = children[i];
                }

                dofIndex = GetLimits(children[i].xDrive, dofIndex);
                dofIndex = GetLimits(children[i].yDrive, dofIndex);
                dofIndex = GetLimits(children[i].zDrive, dofIndex);

                if (i == 0)
                {
                    continue;
                }
            
                ChainLength += Vector3.Distance(children[i].transform.position, children[i - 1].transform.position);
            }

            if (dofIndex != LowerLimits.Length)
            {
                Debug.LogError($"Ensure all joints on {name} have limits defined.");
            }

            if (last != null)
            {
                LastJoint = last.transform;
            }
        
            _zeros = new();
            for (int i = 0; i < _home.Count; i++)
            {
                _zeros.Add(0);
            }

            SetMaxSpeeds(maxSpeeds);
            _currentSpeeds = new float[maxSpeeds.Length];
            for (int i = 0; i < _currentSpeeds.Length; i++)
            {
                _currentSpeeds[i] = maxSpeeds[i];
            }

            if (_home.Count != maxSpeeds.Length)
            {
                Debug.LogError($"{name} has {_home.Count} degrees of freedom but {maxSpeeds.Length} speeds defined.");
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
                if (angles[i] / maxSpeeds[i] > time)
                {
                    time = angles[i] / maxSpeeds[i];
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

        public void SetMaxSpeeds(IEnumerable<float> degrees)
        {
            SetMaxSpeedsRadians(DegreesToRadians(degrees.ToList()).ToArray());
        }

        public void SetMaxSpeedsRadians(float[] radians)
        {
            maxSpeeds = radians;
        }

        public List<float> GetJoints()
        {
            List<float> angles = new();
            Root.GetJointPositions(angles);
            return angles;
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

        private int GetLimits(ArticulationDrive drive, int dofIndex)
        {
            if (drive.lowerLimit == 0 && drive.upperLimit == 0)
            {
                return dofIndex;
            }

            LowerLimits[dofIndex] = drive.lowerLimit * Mathf.Deg2Rad;
            UpperLimits[dofIndex] = drive.upperLimit * Mathf.Deg2Rad;
            return ++dofIndex;
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