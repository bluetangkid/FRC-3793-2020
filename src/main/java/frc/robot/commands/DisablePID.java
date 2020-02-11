/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj2.command.CommandBase;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class DisablePID extends CommandBase {
  CANSparkMax motor;
  public DisablePID(CANSparkMax motor) {
    this.motor = motor;
  }

  @Override
  public void end(boolean forced) {
    super.end(forced);
    motor.getPIDController().setReference(0, ControlType.kVelocity);
  }
}
