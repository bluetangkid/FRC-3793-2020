/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.HowitzerSystem;

public class AimCommand extends CommandBase {
  /**
   * Creates a new AimCommand.
   */

   HowitzerSystem m_HowitzerSystem;
   DriveSystem m_DriveSystem;

   
  public AimCommand(HowitzerSystem howitzerSystem, DriveSystem driveSystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_HowitzerSystem = howitzerSystem;
    m_DriveSystem = driveSystem;

    addRequirements(m_HowitzerSystem, m_DriveSystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  turnTo(m_HowitzerSystem.xOffset);
  m_HowitzerSystem.goToAngle(angle);
  
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
