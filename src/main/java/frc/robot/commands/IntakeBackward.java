/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ConveyorSystem;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.ShooterSystem;

public class IntakeBackward extends CommandBase {
  /**
   * Creates a new IntakeBackward.
   */
  ConveyorSystem cSystem;
  ShooterSystem sSystem;
  IntakeSystem iSystem;
  public IntakeBackward(ConveyorSystem cSystem, ShooterSystem sSystem, IntakeSystem iSystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.cSystem = cSystem;
    this.sSystem = sSystem;
    this.iSystem = iSystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    cSystem.setVictor(-Constants.conveyorSpeed, true);
    iSystem.setMotor(Constants.intakeSpeed);
    sSystem.setSpeed(-1000, -1000);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    cSystem.setVictor(0, true);
    iSystem.setMotor(0);
    sSystem.setSpeed(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
