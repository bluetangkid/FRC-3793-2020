/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.BallStopperSystem;
import frc.robot.subsystems.ConveyorSystem;
import frc.robot.subsystems.IntakeSystem;

public class IntakeCommand extends CommandBase {
  /**
   * Creates a new IntakeCommand.
   */

  private IntakeSystem m_subsystem;
  private BallStopperSystem m_ballStopper;
  private ConveyorSystem C_system;
  public IntakeCommand(IntakeSystem I_System, BallStopperSystem B_system, ConveyorSystem C_system) {
    m_subsystem = I_System;
    this.m_ballStopper = B_system;
    this.C_system = C_system;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(I_System);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ballStopper.setTalon(1);
    m_subsystem.getIntakeMotor().set(ControlMode.PercentOutput, Constants.intakeSpeed);
    C_system.setVictor(Constants.conveyorSpeed);
  }

  public void end(boolean interrupted) {
    super.end(interrupted);
    m_subsystem.getIntakeMotor().set(ControlMode.PercentOutput, 0);
    C_system.setVictor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}