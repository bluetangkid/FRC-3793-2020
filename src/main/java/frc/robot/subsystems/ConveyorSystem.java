/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ConveyorSystem extends SubsystemBase {
  /**
   * Creates a new ConveyorSystem.
   */
  private TalonSRX conveyor;
  private Ultrasonic sonar;

  public ConveyorSystem() {
    sonar = new Ultrasonic(RobotMap.SONAR_TRIG.getPin(), RobotMap.SONAR_ECHO.getPin());
    sonar.setAutomaticMode(true);
    conveyor = new TalonSRX(RobotMap.CONVEYOR_VICTOR.getPin());

    conveyor.configContinuousCurrentLimit(40);
    conveyor.enableCurrentLimit(true);

    
  }

  public void setVictor(double speed, boolean override) {
    if(sonar.getRangeInches() < 8 || override)
      conveyor.set(ControlMode.PercentOutput, speed);
    else conveyor.set(ControlMode.PercentOutput, 0);
    if(System.currentTimeMillis() %10 ==0)
    System.out.println("sonar: " + sonar.getRangeInches());
  }

  public TalonSRX getTalon(){
    return conveyor;
  }
}