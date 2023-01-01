namespace RapidSim.Testers
{
    public class RobotHomeTester : RobotTester
    {
        protected override void Move()
        {
            robot.MoveHome();
        }

        protected override void Snap()
        {
            robot.SnapHome();
        }
    }
}