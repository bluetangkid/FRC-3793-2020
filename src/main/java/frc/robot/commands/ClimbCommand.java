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
  private ClimbSystem climbSystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  Timer timer;
  double dir = 0;
  XboxController controller;
  public ClimbCommand(ClimbSystem climbSystem, XboxController controller) {
    this.climbSystem = climbSystem;
    this.controller = controller;
    this.timer = new Timer();
    timer.start();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climbSystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    dir = controller.getRawAxis(ControllerMap.rightY);
    if(Math.abs(dir) > .1)
    climbSystem.set(Constants.climbSpeed * dir);
    else climbSystem.set(0);
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    climbSystem.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}