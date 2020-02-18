/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.EncoderType;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ShootCommand extends CommandBase {
  CANSparkMax motor;
  double setpoint;
  SimpleMotorFeedforward feedForward;
  public ShootCommand(CANSparkMax motor, double setpoint) {
    this.motor = motor;
    this.setpoint = setpoint;
    feedForward = new SimpleMotorFeedforward(Constants.kSShooter, Constants.kVShooter, Constants.kAShooter);
  }

  public void initialize() {
    motor.getPIDController().setP(Constants.kPShooter);
    motor.getPIDController().setI(0);
    motor.getPIDController().setD(0);
    motor.getPIDController().setReference(-setpoint, ControlType.kVelocity, 0, feedForward.calculate(0));
    motor.getPIDController().setFeedbackDevice(motor.getEncoder(EncoderType.kHallSensor, 42));
    this.schedule();
  }

  public void execute() {
    super.execute();
    motor.getPIDController().setReference(-setpoint, ControlType.kVelocity, 0, feedForward.calculate(motor.getEncoder().getVelocity()));
  }
}