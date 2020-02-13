/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.FindPath;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.HowitzerSystem;

public class AimCommand extends CommandBase {
  /**
   * Creates a new AimCommand.
   */
  //TODO a single button can do this in parallel
   HowitzerSystem m_HowitzerSystem;
   DriveSystem m_DriveSystem;

   double calculatedAngle;

   
  public AimCommand(HowitzerSystem howitzerSystem, DriveSystem driveSystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_HowitzerSystem = howitzerSystem;
    m_DriveSystem = driveSystem;

    addRequirements(m_HowitzerSystem, m_DriveSystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_DriveSystem.setDefaultCommand(new FollowPath(m_DriveSystem, FindPath.getTurn(m_HowitzerSystem.xOffset)));// is this right?
    m_HowitzerSystem.goToAngle(calculatedAngle);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    m_DriveSystem.setDefaultCommand();
    return true;
  }
}
