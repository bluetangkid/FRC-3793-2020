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
    // Drivetrain
    public final static double trackWidth = 1.88797;
    public final static double b = .7;
    public final static double zeta = .5;
    public final static double turnMax = .3;
    public final static double throttleMax = 1;
    public final static double maxVelocity = 15;
    public final static double driveDeadzone = .12f;
    public final static double kPdt = 3.05;
    public final static double kSdt = 0.138;
    public final static double kVdt = 0.817;
    public final static double kAdt = 0.118;

    // Howitzer
    /*
     * \ \ /\ u / \ / \ pivotLen / \ / \ | | dy
     */
    // angle = acos((d^2 + pivotLen^2 - pivotLen^2)/(2*d*pivotLen)) + atan2(dy, dist
    // along lead screw to pivot)
    public final static double pivotLen = 12.5; // in "
    public final static double HowU = 8.5; // in "
    public final static double maxLen = 20.5;
    public final static double rotPerIn = 8;
    public final static double howMin = .25; // in "
    public final static double howMax = 10.25;

    // Shooter
    public final static double kVShooter = 0.126;
    public final static double kSShooter = 0.111;
    public final static double kAShooter = 0.0168;
    public final static double kVShooterB = 0.127;
    public final static double kSShooterB = 0.109;
    public final static double kAShooterB = 0.0167;
    public final static double kPShooter = 0.000000000001;
    public final static double shooterSpeedT = 87;
    public final static double shooterSpeedB = 60;

    // Misc
    public final static int timeoutMs = 500;
    public final static double intakeSpeed = 1;
    public final static double conveyorSpeed = 1;
    public final static double colorWheelSpeed = .8;
    public final static double climbSpeed = 1;
    public final static double climbWaitTime = 1; // number of seconds we wait to climb after start of teleop

}