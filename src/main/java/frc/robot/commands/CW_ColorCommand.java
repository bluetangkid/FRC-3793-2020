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

public class CW_ColorCommand extends CommandBase {
  /**
   * Creates a new CW_PositionCommand.
   */
  ColorWheelSystem cwSystem;
  String specifiedColor = DriverStation.getInstance().getGameSpecificMessage();;
  Color8Bit desiredColor;


  public CW_ColorCommand(ColorWheelSystem cwSystem) {
    this.cwSystem = cwSystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(cwSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    desiredColor = cwSystem.RED;
    switch (specifiedColor) {
      case "B":
        desiredColor = cwSystem.BLUE;
      case "G":
        desiredColor = cwSystem.GREEN;
      case "Y":
        desiredColor = cwSystem.YELLOW;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!cwSystem.isColor(desiredColor)) {
      cwSystem.setColorWheel(.2);
    }
  }

  @Override
  public void end(boolean b) {
    super.end(b);
    cwSystem.setColorWheel(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return cwSystem.isColor(desiredColor);
  }
}