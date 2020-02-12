/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.BallStopperSystem;
import frc.robot.subsystems.ConveyorSystem;
import frc.robot.subsystems.ShooterSystem;

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
  ShooterSystem shooter;
  BallStopperSystem B_system;
  public ConveyorCommand(ConveyorSystem C_system, ShooterSystem shooter, BallStopperSystem B_system) {
    this.C_system = C_system;
    this.shooter = shooter;
    this.B_system = B_system;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(C_system);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    B_system.setTalon(-1);
    if(shooter.mayShoot())
      C_system.setVictor(Constants.conveyorSpeed);
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
