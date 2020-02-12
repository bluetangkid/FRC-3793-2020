/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.HowitzerSystem;

public class moveToHowitzer extends InstantCommand {
  /**
   * Creates a new moveToHowitzer.
   */
   HowitzerSystem m_HowitzerSystem;
   double m_angle;
  public moveToHowitzer(HowitzerSystem howitzerSystem, double angle) {
    m_angle = angle;
    m_HowitzerSystem = howitzerSystem;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_HowitzerSystem.goToAngle(m_angle);
  }
}