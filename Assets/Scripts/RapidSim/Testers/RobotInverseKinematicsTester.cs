namespace RapidSim.Testers
{
    public class RobotInverseKinematicsTester : RobotTester
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
}