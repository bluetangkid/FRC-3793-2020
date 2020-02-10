/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorWheelSystem;

public class ColorWheelCommand extends CommandBase {
  /**
   * Creates a new ColorWheelCommand.
   */
  ColorWheelSystem cw_System;
  int timer = 0;
  int starterColor = cw_System.showColor();
  double spinCount = 0;

  public ColorWheelCommand(ColorWheelSystem cw_System) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(cw_System);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cw_System.setColorWheel(.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer >= 10) {
      timer = 0;
    }
    
    if (timer == 0 && spinCount < 3.5) {
      if (cw_System.showColor() == starterColor) {
        spinCount += .5;
      }
    }
    if (spinCount >= 3.5) {
      cw_System.setColorWheel(0);
    }
    timer++;
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
