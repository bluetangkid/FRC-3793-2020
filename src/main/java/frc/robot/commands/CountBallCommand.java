/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallCountingSystem;

public class CountBallCommand extends CommandBase {
  /**
   * Creates a new CountBallCommand.
   */
  BallCountingSystem mySystem;
  private double lowestVoltage = 10;

  public CountBallCommand(BallCountingSystem system) {
    // Use addRequirements() here to declare subsystem dependencies.
    mySystem = system;
    addRequirements(system);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double voltage = mySystem.getBallCounter().getVoltage();
    if (voltage < lowestVoltage) {
      lowestVoltage = voltage;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
