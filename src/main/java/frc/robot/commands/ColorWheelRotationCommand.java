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
  ColorWheelSystem cwSystem;
  int timer = 0;
  int starterColor;
  double spinCount = 0;

  public ColorWheelRotationCommand(ColorWheelSystem cwSystem) {
    this.cwSystem = cwSystem;
    starterColor = cwSystem.showColor();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(cwSystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer >= 10) {
      timer = 0;
    }

    if (timer == 0 && spinCount < 3.5) {
      if (cwSystem.showColor() == starterColor) {
        spinCount += .5;
      }
    }
    if (spinCount >= 3.5) {
      cwSystem.setColorWheel(0);
    } else
      cwSystem.setColorWheel(.5);
    timer++;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return spinCount >= 3.5;
  }
}