/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
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
  private AHRS navx;

  private double turnOffset;

  public DriveSystem() {
    feedForward = new SimpleMotorFeedforward(Constants.kSdt, Constants.kVdt, Constants.kAdt);
    leftMotorOne = new CANSparkMax(RobotMap.LEFT_DRIVE_MOTOR_ONE.getPin(), MotorType.kBrushless);
    leftMotorTwo = new CANSparkMax(RobotMap.LEFT_DRIVE_MOTOR_TWO.getPin(), MotorType.kBrushless);
    rightMotorOne = new CANSparkMax(RobotMap.RIGHT_DRIVE_MOTOR_ONE.getPin(), MotorType.kBrushless);
    rightMotorTwo = new CANSparkMax(RobotMap.RIGHT_DRIVE_MOTOR_TWO.getPin(), MotorType.kBrushless);
    leftMotorOne.restoreFactoryDefaults();
    leftMotorTwo.restoreFactoryDefaults();
    rightMotorOne.restoreFactoryDefaults();
    rightMotorTwo.restoreFactoryDefaults();
    getLeftMotorOne().getPIDController().setP(Constants.kPdt);
    getLeftMotorOne().getPIDController().setI(0);
    getLeftMotorOne().getPIDController().setD(0);
    getRightMotorOne().getPIDController().setP(Constants.kPdt);
    getRightMotorOne().getPIDController().setI(0);
    getRightMotorOne().getPIDController().setD(0);
    getLeftMotorOne().getPIDController().setFeedbackDevice(getLeftMotorOne().getEncoder(EncoderType.kHallSensor, 42));
    getRightMotorOne().getPIDController().setFeedbackDevice(getRightMotorOne().getEncoder(EncoderType.kHallSensor, 42));
    getLeftMotorTwo().follow(getLeftMotorOne());
    getRightMotorTwo().follow(getRightMotorOne());
    getLeftMotorOne().setSmartCurrentLimit(40);
    getLeftMotorTwo().setSmartCurrentLimit(40);
    getRightMotorOne().setSmartCurrentLimit(40);
    getRightMotorTwo().setSmartCurrentLimit(40);
    navx = new AHRS(Port.kMXP);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("leftWheel", leftMotorOne.getEncoder(EncoderType.kQuadrature, 2048).getPosition());
    //SmartDashboard.putNumber("rightWheel", rightMotorOne.getEncoder(EncoderType.kQuadrature, 2048).getPosition());
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

  public Pose2d getAngle(){
    return new Pose2d(0, 0, new Rotation2d(navx.getAngle()));
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