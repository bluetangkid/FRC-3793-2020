/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSystem;

public class IntakePivotCommand extends CommandBase {
  /**
   * Creates a new IntakePivotCommand.
   */
  IntakeSystem intakeSystem;
  double Speed;
  XboxController controller;

  public IntakePivotCommand(IntakeSystem i, XboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.controller = controller;
    intakeSystem = i;

    addRequirements(i);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSystem.getPivot().set(-.3);
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSystem.getPivot().set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
