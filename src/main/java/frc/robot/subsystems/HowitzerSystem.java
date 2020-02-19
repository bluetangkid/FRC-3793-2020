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

  public HowitzerSystem() {
    aimTalon = new TalonSRX(RobotMap.AIM_TALON.getPin());
    maxLimitSwitch = new DigitalInput(RobotMap.MAX_LIMIT_SWITCH.getPin());
    minLimitSwitch = new DigitalInput(RobotMap.MIN_LIMIT_SWITCH.getPin());
  }

  @Override
  public void periodic() {
    double dist = aimTalon.getSelectedSensorPosition(0)*Constants.tickPerIn;//TODO needs to be track len - tickperin thing + dist between end of track and pivot on x
    howitzerAngle = Math.toDegrees(Math.acos((dist*dist + Constants.pivotLen*Constants.pivotLen - Constants.HowU*Constants.HowU)/(2*dist*Constants.pivotLen)) + Math.atan2(Constants.HowDy, dist));
    if(maxLimitSwitch.get()) aimTalon.set(ControlMode.PercentOutput, -.2);// the stuff following is a simple PF loop
    else if (minLimitSwitch.get()) aimTalon.set(ControlMode.PercentOutput, .2);
    else if (targetDistance - dist > .1) aimTalon.set(ControlMode.PercentOutput, (targetDistance-dist)*.05 + Math.copySign(.15, targetDistance-dist));//just do a P loop for this
    else aimTalon.set(ControlMode.PercentOutput, 0);
  }

  public void goToAngle(double angle) {
    targetDistance = 0 + aimOffset;//TODO Math for this using law of cosines
  }

  public void addOffset() {
    aimOffset += .5;
  }

  public void subOffset() {
    aimOffset -= .5;
  }
}