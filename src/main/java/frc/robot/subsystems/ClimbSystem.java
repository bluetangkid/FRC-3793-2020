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

public class ClimbSystem extends SubsystemBase {
  /**
   * Creates a new ClimbSystem.
   */
  CANSparkMax climbMotor;
  public ClimbSystem() {
  climbMotor = new CANSparkMax(RobotMap.CLIMB_MOTOR.getPin(), MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void set(double d) {
    climbMotor.set(d);
  }
  public CANSparkMax getClimbMotor(){
return climbMotor;
  }
}
