/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ConveyorSystem extends SubsystemBase {
  /**
   * Creates a new ConveyorSystem.
   */
  private VictorSPX conveyorVictor;
  private Ultrasonic sonar;

  public ConveyorSystem() {
    sonar = new Ultrasonic(RobotMap.SONAR_TRIG.getPin(), RobotMap.SONAR_TRIG.getPin());
    conveyorVictor = new VictorSPX(RobotMap.CONVEYOR_VICTOR.getPin());
  }

  public void setVictor(double speed) {
    if(sonar.getRangeInches() < 3)
      conveyorVictor.set(ControlMode.PercentOutput, speed);
  }
}