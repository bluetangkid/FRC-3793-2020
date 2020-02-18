/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class BallCountingSystem extends SubsystemBase {
  /**
   * Creates a new BallCountingSystem.
   */
  private AnalogInput ballCounter;

  public BallCountingSystem() {
    ballCounter = new AnalogInput(RobotMap.BALL_COUNTER.getPin());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public AnalogInput getBallCounter() {
    return ballCounter;
  }
}