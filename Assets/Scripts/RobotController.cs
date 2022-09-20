using UnityEngine;

public class RobotController : MonoBehaviour
{
    [SerializeField]
    private bool solve;

    public RobotData robotData;
    
    public Transform[] joints;

    public Transform tcp;

    public Transform target;
    
    public Vector2[] OriginalAngles { get; private set; }

    private void Start()
    {
        OriginalAngles = new Vector2[joints.Length];
        for (int i = 0; i < joints.Length; i++)
        {
            OriginalAngles[i] = new(joints[i].eulerAngles.x, joints[i].eulerAngles.y);
        }
    }

    private void Update()
    {
        if (!solve)
        {
            return;
        }

        solve = false;

        NeuralNetwork network;

        if (robotData == null || robotData.Data == null)
        {
            network = new(new[] { joints.Length + 3, joints.Length + 3, joints.Length });
        }
        else
        {
            network = robotData.Data[0];
        }
        
        float[] inputs = new float[joints.Length + 3];
        
        Vector3 targetPosition = target.position;

        inputs[0] = targetPosition.x - joints[0].position.x;
        inputs[1] = targetPosition.y - joints[0].position.y;
        inputs[2] = targetPosition.z - joints[0].position.z;
        
        for (int i = 0; i < joints.Length; i++)
        {
            inputs[i + 3] = joints[i].localEulerAngles.z / 360f;
        }
        
        float[] solution = network.Activate(inputs);
        for (int k = 0; k < joints.Length; k++)
        {
            joints[k].localEulerAngles = new(OriginalAngles[k].x, OriginalAngles[k].y, solution[k] * 180);
        }
    }
}