/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

/**
 * Add your docs here.
 */

public class ControllerMap {
    public static XboxController driver = new XboxController(0);
    public static XboxController operator = new XboxController(1);

    public static final int A = 1;
    public static final int B = 2;
    public static final int X = 3;
    public static final int Y = 4;
    public static final int LB = 5;
    public static final int RB = 6;
    public static final int back = 7;
    public static final int start = 8;
    public static final int leftClick = 9;
    public static final int rightClick = 10;

    public static final int leftX = 0;
    public static final int leftY = 1;
    public static final int leftTrigger = 2;
    public static final int rightTrigger = 3;
    public static final int rightX = 4;
    public static final int rightY = 5;
    static public boolean xButtonEnabled = false;
    static public boolean bButtonEnabled = false;
}
