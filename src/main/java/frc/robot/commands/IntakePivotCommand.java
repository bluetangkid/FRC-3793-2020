/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSystem;

public class IntakePivotCommand extends CommandBase {
  /**
   * Creates a new IntakePivotCommand.
   */
  IntakeSystem m_IntakeSystem;
  double Speed;
  public IntakePivotCommand(IntakeSystem i, double s) {
    // Use addRequirements() here to declare subsystem dependencies.
    Speed = s;
    m_IntakeSystem = i;

    addRequirements(i);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_IntakeSystem.getSpark().set(Speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSystem.getSpark().set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
