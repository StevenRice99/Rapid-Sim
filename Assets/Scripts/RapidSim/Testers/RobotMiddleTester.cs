namespace RapidSim.Testers
{
    public class RobotMiddleTester : RobotTester
    {
        protected override void Move()
        {
            robot.MoveMiddle();
        }

        protected override void Snap()
        {
            robot.SnapMiddle();
        }
    }
}