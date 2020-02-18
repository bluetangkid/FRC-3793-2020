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
    public final static double kVShooter = 0.127;
    public final static double kSShooter = 0.093;
    public final static double kAShooter = 0.0101;
    public final static double trackWidth = 0;
    public final static double b = .7;
    public final static double zeta = .5;
    public final static double turnMax = .7;
    public final static double throttleMax = 1;
    public final static double maxVelocity = 40;
    public final static double kPdt = 0.955;
    public final static double kSdt = 0.155;
    public final static double kVdt = 0.0805;
    public final static double kAdt = 0.0129;
    public final static int timeoutMs = 500;
    public final static double kPHow = 0;
    public final static double kIHow = 0;
    public final static double kDHow = 0;
    public final static double kPShooter = 9.29;
    public final static double shooterSpeed = 3600;
    public final static double intakeSpeed = .8;
    public final static double conveyorSpeed = .8;
    public final static double colorWheelSpeed = .8;
    public final static double climbSpeed = 1;
}