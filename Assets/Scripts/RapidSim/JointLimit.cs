namespace RapidSim
{
    public struct JointLimit
    {
        public float Lower;

        public float Upper;

        public JointLimit(float lower, float upper)
        {
            Lower = lower;
            Upper = upper;
        }
    }
}