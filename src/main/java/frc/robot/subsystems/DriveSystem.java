/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.commands.FollowPath;

public class DriveSystem extends SubsystemBase implements DriveBase{
  /**
   * Creates a new DriveSystem.
   */
  private CANSparkMax leftMotorOne;
  private CANSparkMax leftMotorTwo;
  private CANSparkMax rightMotorOne;
  private CANSparkMax rightMotorTwo;

  private FollowPath pathFollower;
  private Pose2d pose;

  public DriveSystem() {
    leftMotorOne = new CANSparkMax(RobotMap.LEFT_DRIVE_MOTOR_ONE.getPin(), MotorType.kBrushless);
    leftMotorTwo = new CANSparkMax(RobotMap.LEFT_DRIVE_MOTOR_TWO.getPin(), MotorType.kBrushless);
    rightMotorOne = new CANSparkMax(RobotMap.RIGHT_DRIVE_MOTOR_ONE.getPin(), MotorType.kBrushless);
    rightMotorTwo = new CANSparkMax(RobotMap.RIGHT_DRIVE_MOTOR_TWO.getPin(), MotorType.kBrushless);
    leftMotorTwo.follow(leftMotorOne);
    rightMotorTwo.follow(rightMotorOne);
    //remember to config the PIDs for all the motors or it won't work
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("leftWheel", leftMotorOne.getEncoder().getPosition());
    SmartDashboard.putNumber("rightWheel", rightMotorOne.getEncoder().getPosition());
    Double[] s = SmartDashboard.getNumberArray("Pose", new Double[3]);
    pose = new Pose2d(new Translation2d(s[0], s[1]), new Rotation2d(s[2]));
  }

  public CANSparkMax getLeftMotorOne() {
    return leftMotorOne;
  }

  public CANSparkMax getLeftMotorTwo() {
    return leftMotorTwo;
  }

  public CANSparkMax getRightMotorOne() {
    return rightMotorOne;
  }

  public CANSparkMax getRightMotorTwo() {
    return rightMotorTwo;
  }

  public Pose2d getPose(){
    return pose;
  }

}
