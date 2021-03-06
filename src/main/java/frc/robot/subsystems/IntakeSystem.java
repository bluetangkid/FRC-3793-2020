/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class IntakeSystem extends SubsystemBase {
  /**
   * Creates a new IntakeSystem.
   */
  private VictorSPX intakeMotor;
  private Spark pivotMotor;

  public IntakeSystem() {
    intakeMotor = new VictorSPX(RobotMap.INTAKE_VICTOR.getPin());
    pivotMotor = new Spark(RobotMap.INTAKE_PIVOT_SPARK.getPin());
  }

  @Override
  public void periodic() {}

  public VictorSPX getIntakeMotor() {
    return intakeMotor;
  }

  public void drop(){
    pivotMotor.set(.5);
  }

  public IntakeSystem setIntake(double d) {
    intakeMotor.set(ControlMode.PercentOutput, d);
    return this;
  }

  public Spark getPivot(){
    return pivotMotor;
  }

}