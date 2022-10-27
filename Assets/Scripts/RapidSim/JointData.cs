using System;
using System.Collections.Generic;
using UnityEngine;

namespace RapidSim
{
    public class JointData
    {
        public ArticulationBody Joint;

        public JointData Child;

        public ArticulationJointType Type => Joint.jointType;

        public bool HasMotion => Type != ArticulationJointType.FixedJoint;

        public bool XMotion => Joint.xDrive.lowerLimit != 0 && Joint.xDrive.upperLimit != 0;

        public bool YMotion => Joint.yDrive.lowerLimit != 0 && Joint.yDrive.upperLimit != 0;

        public bool ZMotion => Joint.zDrive.lowerLimit != 0 && Joint.zDrive.upperLimit != 0;

        public JointData(ArticulationBody joint, JointData child = null)
        {
            Joint = joint;
            Child = child;
        }

        public List<float> LowerLimits(bool children = true)
        {
            List<float> limits = new();
            switch (Type)
            {
                case ArticulationJointType.FixedJoint:
                    break;
                case ArticulationJointType.PrismaticJoint:
                    limits.Add(
                        XMotion ? Joint.xDrive.lowerLimit :
                        YMotion ? Joint.yDrive.lowerLimit :
                        Joint.zDrive.lowerLimit
                    );
                    break;
                case ArticulationJointType.RevoluteJoint:
                    limits.Add(Joint.xDrive.lowerLimit * Mathf.Deg2Rad);
                    break;
                case ArticulationJointType.SphericalJoint:
                default:
                    if (XMotion)
                    {
                        limits.Add(Joint.xDrive.lowerLimit * Mathf.Deg2Rad);
                    }
                    if (YMotion)
                    {
                        limits.Add(Joint.yDrive.lowerLimit * Mathf.Deg2Rad);
                    }
                    if (ZMotion)
                    {
                        limits.Add(Joint.zDrive.lowerLimit * Mathf.Deg2Rad);
                    }
                    break;
            }

            if (children && Child != null)
            {
                limits.AddRange(Child.LowerLimits());
            }

            return limits;
        }
        
        public List<float> UpperLimits(bool children = true)
        {
            List<float> limits = new();
            switch (Type)
            {
                case ArticulationJointType.FixedJoint:
                    break;
                case ArticulationJointType.PrismaticJoint:
                    limits.Add(
                        XMotion ? Joint.xDrive.upperLimit :
                        YMotion ? Joint.yDrive.upperLimit :
                        Joint.zDrive.upperLimit
                    );
                    break;
                case ArticulationJointType.RevoluteJoint:
                    limits.Add(Joint.xDrive.upperLimit * Mathf.Deg2Rad);
                    break;
                case ArticulationJointType.SphericalJoint:
                default:
                    if (XMotion)
                    {
                        limits.Add(Joint.xDrive.upperLimit * Mathf.Deg2Rad);
                    }
                    if (YMotion)
                    {
                        limits.Add(Joint.yDrive.upperLimit * Mathf.Deg2Rad);
                    }
                    if (ZMotion)
                    {
                        limits.Add(Joint.zDrive.upperLimit * Mathf.Deg2Rad);
                    }
                    break;
            }

            if (children && Child != null)
            {
                limits.AddRange(Child.UpperLimits());
            }

            return limits;
        }
    }
}