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
import frc.robot.subsystems.ConveyorSystem;
import frc.robot.subsystems.IntakeSystem;

public class IntakeCommand extends CommandBase {
  /**
   * Creates a new IntakeCommand.
   */

  private IntakeSystem intakeSystem;
  private ConveyorSystem conveyorSystem;
  public IntakeCommand(IntakeSystem intakeSystem, ConveyorSystem conveyorSystem) {
    this.intakeSystem = intakeSystem;
    this.conveyorSystem = conveyorSystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSystem.getIntakeMotor().set(ControlMode.PercentOutput, -Constants.intakeSpeed);
    conveyorSystem.setVictor(Constants.conveyorSpeed, false);
  }

  public void end(boolean interrupted) {
    super.end(interrupted);
    intakeSystem.getIntakeMotor().set(ControlMode.PercentOutput, 0);
    conveyorSystem.setVictor(0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}