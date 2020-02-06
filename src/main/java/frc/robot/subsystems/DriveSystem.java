/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class DriveSystem extends SubsystemBase {
  /**
   * Creates a new DriveSystem.
   */
  CANSparkMax leftMotorOne;
  CANSparkMax leftMotorTwo;
  CANSparkMax rightMotorOne;
  CANSparkMax rightMotorTwo;
  SpeedControllerGroup left;
  SpeedControllerGroup right;
  public DifferentialDrive drive;
  
  public DriveSystem() {
    leftMotorOne = new CANSparkMax(RobotMap.LEFT_DRIVE_MOTOR_ONE.getPin(), MotorType.kBrushless);
    leftMotorTwo = new CANSparkMax(RobotMap.LEFT_DRIVE_MOTOR_TWO.getPin(), MotorType.kBrushless);
    rightMotorOne = new CANSparkMax(RobotMap.RIGHT_DRIVE_MOTOR_ONE.getPin(), MotorType.kBrushless);
    rightMotorTwo = new CANSparkMax(RobotMap.RIGHT_DRIVE_MOTOR_TWO.getPin(), MotorType.kBrushless);
    left = new SpeedControllerGroup(leftMotorOne, leftMotorTwo);
    right = new SpeedControllerGroup(rightMotorOne,rightMotorTwo);
    drive = new DifferentialDrive(left, right);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
