/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class ShooterSystem extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */
  CANSparkMax topWheel;
  CANSparkMax bottomWheel;
  CANPIDController top, bottom;
  CANEncoder topE, botE;
  SimpleMotorFeedforward feedForward;

  public ShooterSystem() {
    topWheel = new CANSparkMax(RobotMap.TOP_SHOOTER.getPin(), MotorType.kBrushless);
    bottomWheel = new CANSparkMax(RobotMap.BOTTOM_SHOOTER.getPin(), MotorType.kBrushless);
    feedForward = new SimpleMotorFeedforward(Constants.kSShooter, Constants.kVShooter, Constants.kAShooter);
    top = topWheel.getPIDController();
    bottom = bottomWheel.getPIDController();
    topE = topWheel.getEncoder();
    botE = bottomWheel.getEncoder();
    top.setP(Constants.kPShooter);
    top.setI(0);
    top.setD(0);
    bottomWheel.setIdleMode(IdleMode.kCoast);
    topWheel.setInverted(true);

    bottom.setP(Constants.kPShooter);
    bottom.setI(0);
    bottom.setD(0);
    topWheel.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public CANSparkMax getTopWheel() {
    return topWheel;
  }

  public CANSparkMax getBottomWheel() {
    return bottomWheel;
  }

  public void setSpeed(double t, double b){
    bottom.setReference(b, ControlType.kVelocity, 0, feedForward.calculate(b));
    top.setReference(t, ControlType.kVelocity, 0, feedForward.calculate(t));
  }

  public boolean mayShoot() {
    System.out.println(topE.getVelocity());
    return Math.abs(topE.getVelocity()/60f) > .95 * Math.abs(Constants.shooterSpeedT) && Math.abs(botE.getVelocity()/60f) > .95 * Math.abs(Constants.shooterSpeedB);
  }
}
