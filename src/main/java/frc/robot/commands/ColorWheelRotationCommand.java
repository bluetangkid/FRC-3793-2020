/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorWheelSystem;

public class ColorWheelRotationCommand extends CommandBase {
  /**
   * Creates a new ColorWheelCommand.
   */
  ColorWheelSystem cw_System;
  int timer = 0;
  int starterColor = cw_System.showColor();
  double spinCount = 0;

  public ColorWheelRotationCommand(ColorWheelSystem cw_System) {
    this.cw_System = cw_System;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(cw_System);
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
    } else
      cw_System.setColorWheel(.5);
    timer++;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
