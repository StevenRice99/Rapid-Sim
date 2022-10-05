﻿using System;
using UnityEngine;

public class RobotSolveTester : MonoBehaviour
{
    [SerializeField]
    private RobotAgent robot;

    [SerializeField]
    private bool move;

    [SerializeField]
    private bool snap;

    private void Update()
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