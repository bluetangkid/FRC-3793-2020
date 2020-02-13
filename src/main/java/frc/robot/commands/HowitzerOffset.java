/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HowitzerSystem;

public class HowitzerOffset extends CommandBase {
  /**
   * Creates a new HowitzerOffset.
   */
  private HowitzerSystem m_subSystem;
  private double change;

  public HowitzerOffset(HowitzerSystem H_System, double degrees) {
    m_subSystem = H_System;
    change = degrees;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(H_System);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subSystem.setyOffset(m_subSystem.getYOffset() + change);
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
