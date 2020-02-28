/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SerialPort.Port;
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
  private AHRS gyro;

  public DigitalInput maxLimitSwitch;
  public DigitalInput minLimitSwitch;

  NetworkTable limelightTable;
  NetworkTableEntry horizontalOffset;

  double currentHowitzerPosition;

  final double lengthOfHowitzerIn = 40;
  public double howitzerAngle;
  public double targetAngle;
  JoystickButton in, out;

  final double Kp = .1;

  public HowitzerSystem(JoystickButton in, JoystickButton out) {
    this.in = in;
    this.out = out;
    aimTalon = new TalonSRX(RobotMap.AIM_TALON.getPin());
    maxLimitSwitch = new DigitalInput(RobotMap.MAX_LIMIT_SWITCH.getPin());
    minLimitSwitch = new DigitalInput(RobotMap.MIN_LIMIT_SWITCH.getPin());

    aimTalon.configContinuousCurrentLimit(40);
    gyro = new AHRS(Port.kUSB);
  }

  @Override
  public void periodic() {
    howitzerAngle = gyro.getPitch();
    if(System.currentTimeMillis() %10 ==0)
    System.out.println("gyro: " + howitzerAngle);
    
    if(!maxLimitSwitch.get()) aimTalon.set(ControlMode.PercentOutput, 0.2);// the stuff following is a simple PF loop
    else if (!minLimitSwitch.get()) aimTalon.set(ControlMode.PercentOutput, -.2);
    else if(in.get()) aimTalon.set(ControlMode.PercentOutput, -.5);
    else if(out.get()) aimTalon.set(ControlMode.PercentOutput, .5);
    //else if (targetAngle - howitzerAngle > .1) aimTalon.set(ControlMode.PercentOutput, Kp*(targetAngle - howitzerAngle));//just do a P loop for this
    else aimTalon.set(ControlMode.PercentOutput, 0);
  }

  public void goToAngle(double angle) {
    targetAngle = angle;
  }

  public void addOffset() {
    aimOffset += .5;
  }

  public void subOffset() {
    aimOffset -= .5;
  }
}