using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

namespace RapidSim
{
    [DisallowMultipleComponent]
    [RequireComponent(typeof(ArticulationBody))]
    public class RobotJoint : MonoBehaviour
    {
        [Tooltip("The speed in meters per second (for prismatic joints) or in degrees per second for each joint axis.")]
        [SerializeField]
        private float3 speed;
        
        public ArticulationBody Joint { get; private set; }
        
        public JointLimit LimitX { get; private set; }
        
        public JointLimit LimitY { get; private set; }
        
        public JointLimit LimitZ { get; private set; }

        public bool HasMotion => Type != ArticulationJointType.FixedJoint;

        public bool XMotion => XDrive.lowerLimit != 0 && XDrive.upperLimit != 0;

        public bool YMotion => YDrive.lowerLimit != 0 && YDrive.upperLimit != 0;

        public bool ZMotion => ZDrive.lowerLimit != 0 && ZDrive.upperLimit != 0;

        public float SpeedX => Type == ArticulationJointType.PrismaticJoint ? speed.x : speed.x * Mathf.Deg2Rad;

        public float SpeedY => Type == ArticulationJointType.PrismaticJoint ? speed.y : speed.y * Mathf.Deg2Rad;

        public float SpeedZ => Type == ArticulationJointType.PrismaticJoint ? speed.z : speed.z * Mathf.Deg2Rad;

        public ArticulationJointType Type => Joint.jointType;

        private ArticulationDrive XDrive => Joint.xDrive;

        private ArticulationDrive YDrive => Joint.yDrive;

        private ArticulationDrive ZDrive => Joint.zDrive;

        private void Start()
        {
            OnValidate();
            
            switch (Type)
            {
                case ArticulationJointType.FixedJoint:
                    break;
                case ArticulationJointType.PrismaticJoint:
                    LimitX = new(XDrive.lowerLimit, XDrive.upperLimit);
                    LimitY = new(YDrive.lowerLimit, YDrive.upperLimit);
                    LimitZ = new(ZDrive.lowerLimit, ZDrive.upperLimit);
                    break;
                case ArticulationJointType.RevoluteJoint:
                    LimitX = new(XDrive.lowerLimit * Mathf.Deg2Rad, XDrive.upperLimit * Mathf.Deg2Rad);
                    LimitY = new(0, 0);
                    LimitZ = new(0, 0);
                    break;
                case ArticulationJointType.SphericalJoint:
                default:
                    if (XMotion)
                    {
                        LimitX = new(XDrive.lowerLimit * Mathf.Deg2Rad, XDrive.upperLimit * Mathf.Deg2Rad);
                    }
                    else
                    {
                        LimitX = new(0, 0);
                    }
                    if (YMotion)
                    {
                        LimitY = new(YDrive.lowerLimit * Mathf.Deg2Rad, YDrive.upperLimit * Mathf.Deg2Rad);
                    }
                    else
                    {
                        LimitY = new(0, 0);
                    }
                    if (ZMotion)
                    {
                        LimitY = new(ZDrive.lowerLimit * Mathf.Deg2Rad, ZDrive.upperLimit * Mathf.Deg2Rad);
                    }
                    else
                    {
                        LimitZ = new(0, 0);
                    }
                    break;
            }
        }

        private void OnValidate()
        {
            Joint = GetComponent<ArticulationBody>();
            if (Joint == null)
            {
                return;
            }

            ArticulationDrive drive = XDrive;
            drive.stiffness = 100000;
            drive.damping = 100;
            drive.forceLimit = float.MaxValue;
            drive.targetVelocity = 0;
            Joint.xDrive = drive;
            
            drive = YDrive;
            drive.stiffness = 100000;
            drive.damping = 100;
            drive.forceLimit = float.MaxValue;
            drive.targetVelocity = 0;
            Joint.yDrive = drive;
            
            drive = ZDrive;
            drive.stiffness = 100000;
            drive.damping = 100;
            drive.forceLimit = float.MaxValue;
            drive.targetVelocity = 0;
            Joint.zDrive = drive;
        }

        public List<JointLimit> Limits()
        {
            List<JointLimit> limits = new();
            if (XMotion)
            {
                limits.Add(LimitX);
            }
            if (YMotion)
            {
                limits.Add(LimitY);
            }
            if (ZMotion)
            {
                limits.Add(LimitZ);
            }

            return limits;
        }
    }
}