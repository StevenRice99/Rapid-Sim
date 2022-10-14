public class RobotInverseKinematicsTester : RobotHomeTester
{
    protected override void Update()
    {
        if (move)
        {
            move = false;
            robot.Move(transform);
        }

        if (snap)
        {
            snap = false;
            robot.Snap(transform);
        }
    }
}