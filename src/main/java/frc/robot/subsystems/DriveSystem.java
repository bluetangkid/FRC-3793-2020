/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
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
  private CANPIDController left, right, left2, right2;
  private CANEncoder encL, encR;
  private SimpleMotorFeedforward feedForward;
  private DifferentialDriveOdometry odometry;
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
    rightMotorOne.restoreFactoryDefaults();
    leftMotorTwo.restoreFactoryDefaults();
    rightMotorTwo.restoreFactoryDefaults();
    
    left = leftMotorOne.getPIDController();
    right = rightMotorOne.getPIDController();
    left2 = leftMotorTwo.getPIDController();
    right2 = rightMotorTwo.getPIDController();
    encL = leftMotorOne.getEncoder();
    encR = rightMotorOne.getEncoder();
    //encL = leftMotorOne.getEncoder(EncoderType.kQuadrature, 8192);
    //encR = rightMotorOne.getEncoder(EncoderType.kQuadrature, 8192);
    //left.setFeedbackDevice(encL);
    //right.setFeedbackDevice(encR);

    left.setP(Constants.kPdt);
    left.setI(0);
    left.setD(0);
    right.setP(Constants.kPdt);
    right.setI(0);
    right.setD(0);
    left2.setP(Constants.kPdt);
    left2.setI(0);
    left2.setD(0);
    right2.setP(Constants.kPdt);
    right2.setI(0);
    right2.setD(0);
    leftMotorOne.setIdleMode(IdleMode.kBrake);
    leftMotorTwo.setIdleMode(IdleMode.kBrake);
    rightMotorOne.setIdleMode(IdleMode.kBrake);
    rightMotorTwo.setIdleMode(IdleMode.kBrake);
    leftMotorOne.setSmartCurrentLimit(50);
    leftMotorTwo.setSmartCurrentLimit(50);
    rightMotorOne.setSmartCurrentLimit(50);
    rightMotorTwo.setSmartCurrentLimit(50);

    leftMotorOne.setClosedLoopRampRate(.5);
    leftMotorTwo.setClosedLoopRampRate(.5);
    rightMotorOne.setClosedLoopRampRate(.5);
    rightMotorTwo.setClosedLoopRampRate(.5);
    
    odometry = new DifferentialDriveOdometry(new Rotation2d(0), new Pose2d(0, 0, new Rotation2d(0)));
    navx = new AHRS(SPI.Port.kMXP);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("leftWheel", leftMotorOne.getEncoder(EncoderType.kQuadrature, 2048).getPosition());
    //SmartDashboard.putNumber("rightWheel", rightMotorOne.getEncoder(EncoderType.kQuadrature, 2048).getPosition());
    //Double[] s = SmartDashboard.getNumberArray("Pose", new Double[3]);
    //pose = new Pose2d(new Translation2d(s[0], s[1]), new Rotation2d(s[2]));
    odometry.update(new Rotation2d(navx.getAngle()), encL.getPosition()*.1524*Math.PI, encR.getPosition()*.1524*Math.PI);//TODO what is gearbox reduction
  }

  public double getAngle(){
    return navx.getAngle();
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

  public Pose2d getPose() {//TODO if it disagrees with the t265 by .25m, reset pose to t265
    return odometry.getPoseMeters();
  }

  public SimpleMotorFeedforward getFF(){
    return feedForward;
  }
  public double getSpeedAway() {
    double spe = Math.sqrt(Math.pow(encL.getVelocity(), 2) + Math.pow(encR.getVelocity(), 2));
    return spe*Math.cos(navx.getAngle()+90);//TODO gearing + sign for it
  }

  public void setMotorVelocity(double leftV, double rightV) {//TODO find the gearing multiplier. assume we are passed linear m/s speed and convert to motor rotational speed
    left.setReference(-leftV, ControlType.kVelocity, 0, feedForward.calculate(leftV));//might need to divide by (60f*Constants.maxVelocity)
    right.setReference(rightV, ControlType.kVelocity, 0, feedForward.calculate(rightV));
    left2.setReference(-leftV, ControlType.kVelocity, 0, feedForward.calculate(leftV));
    right2.setReference(rightV, ControlType.kVelocity, 0, feedForward.calculate(rightV));
  }

  public void diss(){
    leftMotorOne.set(0);
    rightMotorOne.set(0);
    leftMotorTwo.set(0);
    rightMotorTwo.set(0);
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