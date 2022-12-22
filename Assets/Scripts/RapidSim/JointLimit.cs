namespace RapidSim
{
    public struct JointLimit
    {
        public readonly float lower;

        public readonly float upper;

        public JointLimit(float lower, float upper)
        {
            this.lower = lower;
            this.upper = upper;
        }
    }
}