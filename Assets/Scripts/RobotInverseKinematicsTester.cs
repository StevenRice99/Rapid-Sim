public class RobotInverseKinematicsTester : RobotHomeTester
{
    protected override void Move()
    {
        robot.Move(transform);
    }

    protected override void Snap()
    {
        robot.Snap(transform);
    }
}