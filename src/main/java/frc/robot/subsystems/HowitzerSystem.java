/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class HowitzerSystem extends SubsystemBase {
  /**
   * Creates a new HowitzerSystem.
   */
  public TalonSRX aimTalon;
  private double aimOffset;

  public DigitalInput maxLimitSwitch;
  public DigitalInput minLimitSwitch;

  NetworkTable limelightTable;
  NetworkTableEntry horizontalOffset;

  double currentHowitzerPosition;

  final double lengthOfHowitzerIn = 40;
  public double howitzerAngle;
  public double targetDistance;
  JoystickButton in, out;
  public int zero;

  public HowitzerSystem(JoystickButton in, JoystickButton out) {
    this.in = in;
    this.out = out;
    aimTalon = new TalonSRX(RobotMap.AIM_TALON.getPin());
    maxLimitSwitch = new DigitalInput(RobotMap.MAX_LIMIT_SWITCH.getPin());
    minLimitSwitch = new DigitalInput(RobotMap.MIN_LIMIT_SWITCH.getPin());

    aimTalon.configContinuousCurrentLimit(40);
    zero = -aimTalon.getSelectedSensorPosition(0);
    targetDistance = zero+aimTalon.getSelectedSensorPosition(0);
    
  }

  @Override
  public void periodic() {
    double dist = ((aimTalon.getSelectedSensorPosition(0)+zero)*Constants.rotPerIn)/((25d*(36d/24d))/4096d);//TODO needs to be track len - tickperin thing + dist between end of track and pivot on x
    if(!maxLimitSwitch.get()) {
      aimTalon.set(ControlMode.PercentOutput, 0.2);// the stuff following is a simple PF loop
      zero = -aimTalon.getSelectedSensorPosition(0);
    }
    else if (!minLimitSwitch.get()) {
      aimTalon.set(ControlMode.PercentOutput, -.2);
    }
    else if(in.get()) {
      aimTalon.set(ControlMode.PercentOutput, -.5);
    }
    else if(out.get()) aimTalon.set(ControlMode.PercentOutput, .5);
    //else if (targetDistance - dist > .1) aimTalon.set(ControlMode.PercentOutput, (targetDistance-dist)*.05 + Math.copySign(.15, targetDistance-dist));//just do a P loop for this
    else aimTalon.set(ControlMode.PercentOutput, 0);
    System.out.println(dist + ":" + targetDistance);
  }

  public void goToAngle(double angle) {
    double angie = Math.PI - (Math.cos((Constants.HowU/Constants.pivotLen)*Math.sin(Math.toRadians(angle)))+Math.toRadians(angle));
    targetDistance = (228.5-212.5*Math.cos(angie + aimOffset));
  }

  public void addOffset() {
    aimOffset += .5;
  }

  public void subOffset() {
    aimOffset -= .5;
  }
}