/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    //Drivetrain
    public final static double trackWidth = 1.888;
    public final static double b = .7;
    public final static double zeta = .5;
    public final static double turnMax = .7;
    public final static double throttleMax = 1;
    public final static double maxVelocity = 40;
    public final static double driveDeadzone = .1f;
    public final static double kPdt = 3.32;
    public final static double kSdt = 0.138;
    public final static double kVdt = 0.817;
    public final static double kAdt = 0.118;

    //Howitzer
    /*       \
    *         \
    *         /\
    *   u   /   \
    *     /      \  pivotLen
    *   /         \
    * /            \  
    *              |
    *              | dy     */
    // angle = acos((d^2 + pivotLen^2 - pivotLen^2)/(2*d*pivotLen)) + atan2(dy, dist along lead screw to pivot)
    public final static double pivotLen = 5; // in "
    public final static double HowU = 5; //in "
    public final static double HowDy = 1; //in "
    public final static double tickPerIn = 2;
    public final static double howMin = .25; //in "
    public final static double howMax = 5;

    //Shooter
    public final static double kVShooter = 0.129;
    public final static double kSShooter = 0.0497;
    public final static double kAShooter = 0.0154;
    public final static double kPShooter = 1.17;
    public final static double shooterSpeedT = 100;
    public final static double shooterSpeedB = -120;

    //Misc
    public final static int timeoutMs = 500;
    public final static double intakeSpeed = .8;
    public final static double conveyorSpeed = .5;
    public final static double colorWheelSpeed = .8;
    public final static double climbSpeed = 1;
}