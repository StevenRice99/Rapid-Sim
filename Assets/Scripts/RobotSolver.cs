using System.Collections.Generic;
using UnityEngine;

[DisallowMultipleComponent]
[RequireComponent(typeof(RobotController))]
public class RobotSolver : MonoBehaviour
{
    private RobotController _robotController;

    private void Start()
    {
        _robotController = GetComponent<RobotController>();
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
        List<float> joints = JointOutputs(Solve(PrepareInputs(_robotController.GetJoints(), position, rotation)));
        
        // TODO: Finalize movement with Hybrid IK.

        return joints;
    }

    public List<float> Solve(List<float> inputs)
    {
        // TODO: Solve with neural network.
        return new();
    }

    public List<float> PrepareInputs(List<float> joints, Vector3 position, Quaternion rotation)
    {
        List<float> inputs = new();
        position = RelativePosition(position);
        inputs.Add(position.x);
        inputs.Add(position.y);
        inputs.Add(position.z);
        rotation = RelativeRotation(rotation);
        inputs.Add(rotation.x);
        inputs.Add(rotation.y);
        inputs.Add(rotation.z);
        inputs.Add(rotation.w);
        inputs.AddRange(JointInputs(joints));
        return inputs;
    }
    
    private List<float> JointInputs(List<float> joints)
    {
        for (int i = 0; i < joints.Count; i++)
        {
            joints[i] = (joints[i] - _robotController.LowerLimits[i]) / (_robotController.UpperLimits[i] - _robotController.LowerLimits[i]);
        }

        return joints;
    }
    
    private List<float> JointOutputs(List<float> joints)
    {
        for (int i = 0; i < joints.Count; i++)
        {
            joints[i] = Mathf.Clamp(joints[i] * (_robotController.UpperLimits[i] - _robotController.LowerLimits[i]) + _robotController.LowerLimits[i], _robotController.LowerLimits[i], _robotController.UpperLimits[i]);
        }

        return joints;
    }

    public Vector3 RelativePosition(Vector3 position) => _robotController.Root.transform.InverseTransformPoint(position);
    
    public Quaternion RelativeRotation(Quaternion rotation) => Quaternion.Inverse(_robotController.Root.transform.rotation) * rotation;
}