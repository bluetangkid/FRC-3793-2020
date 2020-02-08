/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorWheelSystem;

public class CW_PositionCommand extends CommandBase {
  /**
   * Creates a new CW_PositionCommand.
   */
  ColorWheelSystem CW_System;
  String specifiedColor = DriverStation.getInstance().getGameSpecificMessage();;
  Color8Bit desiredColor;

  
  public CW_PositionCommand(ColorWheelSystem CW_System) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(CW_System);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch (specifiedColor)
    {
case "R":
desiredColor = CW_System.RED;
case "B":
desiredColor = CW_System.BLUE;
case "G":
desiredColor = CW_System.GREEN;
case "Y":
desiredColor= CW_System.YELLOW;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(CW_System.isColor(desiredColor))
    {
CW_System.setColorWheel(0);
    }
    else
    {
      CW_System.setColorWheel(.2);
    }
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
