/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ShooterSystem extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */
  CANSparkMax topWheel;
  CANSparkMax bottomWheel;

  public ShooterSystem() {
  topWheel = new CANSparkMax(RobotMap.TOP_SHOOTER.getPin(), MotorType.kBrushless);
  bottomWheel = new CANSparkMax(RobotMap.BOTTOM_SHOOTER.getPin(), MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
