/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ConveyorSystem;

/**
 * An example command that uses an example subsystem.
 */
public class ConveyorCommand extends CommandBase {
  private ConveyorSystem C_system;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ConveyorCommand(ConveyorSystem C_system) {
    this.C_system = C_system;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(C_system);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      C_system.setVictor(.8);
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    C_system.setVictor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
