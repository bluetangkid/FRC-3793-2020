/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSystem;

public class IntakeCommand extends CommandBase {
  /**
   * Creates a new IntakeCommand.
   */

  private IntakeSystem m_subsystem;
  public IntakeCommand(IntakeSystem I_System) {
    m_subsystem = I_System;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(I_System);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override//TODO motors will stay spinning, make doNothing set it to 0 or something
  public void execute() {
    m_subsystem.getIntakeMotor().set(ControlMode.PercentOutput, .8);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
