/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class DriveSystem extends SubsystemBase {
  /**
   * Creates a new DriveSystem.
   */
  private CANSparkMax leftMotorOne;
  private CANSparkMax leftMotorTwo;
  private CANSparkMax rightMotorOne;
  private CANSparkMax rightMotorTwo;
  private SimpleMotorFeedforward feedForward;
  private Pose2d pose;

  private double turnOffset;

  public DriveSystem() {
    feedForward = new SimpleMotorFeedforward(Constants.kSdt, Constants.kVdt, Constants.kAdt);
    leftMotorOne = new CANSparkMax(RobotMap.LEFT_DRIVE_MOTOR_ONE.getPin(), MotorType.kBrushless);
    leftMotorTwo = new CANSparkMax(RobotMap.LEFT_DRIVE_MOTOR_TWO.getPin(), MotorType.kBrushless);
    rightMotorOne = new CANSparkMax(RobotMap.RIGHT_DRIVE_MOTOR_ONE.getPin(), MotorType.kBrushless);
    rightMotorTwo = new CANSparkMax(RobotMap.RIGHT_DRIVE_MOTOR_TWO.getPin(), MotorType.kBrushless);
    leftMotorOne.getPIDController().setP(Constants.kPdt);
    leftMotorOne.getPIDController().setI(0);
    leftMotorOne.getPIDController().setD(0);
    rightMotorOne.getPIDController().setP(Constants.kPdt);
    rightMotorOne.getPIDController().setI(0);
    rightMotorOne.getPIDController().setD(0);
    leftMotorOne.getPIDController().setFeedbackDevice(leftMotorOne.getEncoder(EncoderType.kHallSensor, 42));
    rightMotorOne.getPIDController().setFeedbackDevice(rightMotorOne.getEncoder(EncoderType.kHallSensor, 42));
    leftMotorTwo.follow(leftMotorOne);
    rightMotorTwo.follow(rightMotorOne);
    leftMotorOne.setSmartCurrentLimit(40);
    leftMotorTwo.setSmartCurrentLimit(40);
    rightMotorOne.setSmartCurrentLimit(40);
    rightMotorTwo.setSmartCurrentLimit(40);
    // remember to config the PIDs for all the motors or it won't work
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("leftWheel", leftMotorOne.getEncoder(EncoderType.kQuadrature, 2048).getPosition());
    SmartDashboard.putNumber("rightWheel", rightMotorOne.getEncoder(EncoderType.kQuadrature, 2048).getPosition());
    //Double[] s = SmartDashboard.getNumberArray("Pose", new Double[3]);
    //pose = new Pose2d(new Translation2d(s[0], s[1]), new Rotation2d(s[2]));
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

  public Pose2d getPose() {
    return pose;
  }

  public SimpleMotorFeedforward getFF(){
    return feedForward;
  }

  public void setMotorVelocity(double left, double right) {
    leftMotorOne.getPIDController().setReference(left, ControlType.kVelocity/*, 0, feedForward.calculate(leftMotorOne.getEncoder(EncoderType.kHallSensor, 42).getVelocity())*/);
    rightMotorOne.getPIDController().setReference(right, ControlType.kVelocity/*, 0, feedForward.calculate(rightMotorOne.getEncoder(EncoderType.kHallSensor, 42).getVelocity())*/);
  }
  public void addOffset() {
    turnOffset += 1;
  }

  public void subOffset() {
    turnOffset -= 1;
  }

  public double getAngOffset() {
    return turnOffset;
  }
}