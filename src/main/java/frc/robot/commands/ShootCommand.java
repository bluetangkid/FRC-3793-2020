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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ShootCommand extends InstantCommand {
  CANSparkMax motor;
  double setpoint;
  public ShootCommand(CANSparkMax motor, double setpoint) {
    this.motor = motor;
    this.setpoint = setpoint;
  }

  public void initialize(){
    motor.getPIDController().setP(Constants.kPShooter);
    motor.getPIDController().setI(Constants.kIShooter);
    motor.getPIDController().setD(Constants.kDShooter);
    motor.getPIDController().setReference(setpoint, ControlType.kVelocity);
    motor.getPIDController().setFeedbackDevice(motor.getEncoder(EncoderType.kHallSensor, 42));
    this.schedule();
  }

  public void execute() {
    super.execute();
    System.out.println("Execute");
  }
}
