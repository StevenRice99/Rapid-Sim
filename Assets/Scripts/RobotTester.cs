using UnityEngine;

public abstract class RobotTester : MonoBehaviour
{
    [SerializeField]
    protected bool move;

    [SerializeField]
    protected bool snap;

    protected abstract void Awake();

    protected abstract void Move();

    protected abstract void Snap();

    private void OnValidate()
    {
        Awake();
    }
    
    private void Update()
    {
        if (move)
        {
            move = false;
            Move();
        }

        if (!snap)
        {
            return;
        }

        snap = false;
        Snap();
    }
}