/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.ControllerMap;
import frc.robot.subsystems.ClimbSystem;

/**
 * An example command that uses an example subsystem.
 */
public class ClimbCommand extends CommandBase {
  private ClimbSystem C_system;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  Timer timer;
  double dir = 0;
  XboxController controller;
  public ClimbCommand(ClimbSystem C_system, XboxController controller) {
    this.C_system = C_system;
    this.controller = controller;
    this.timer = new Timer();
    timer.start();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(C_system);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    dir = controller.getRawAxis(ControllerMap.rightY);
    C_system.set(Constants.climbSpeed * dir);
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    C_system.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}