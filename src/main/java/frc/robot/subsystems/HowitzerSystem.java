/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.commands.AimCommand;

public class HowitzerSystem extends SubsystemBase {
  /**
   * Creates a new HowitzerSystem.
   */
  public TalonSRX aimTalon;
  private double aimOffset;
  private PigeonIMU gyro;

  public DigitalInput maxLimitSwitch;
  public DigitalInput minLimitSwitch;

  NetworkTable limelightTable;
  NetworkTableEntry horizontalOffset;

  double currentHowitzerPosition;
  public double howitzerAngle;
  public double targetAngle;
  double presetAngle;
  JoystickButton in, out;
  XboxController operatorController;
  int targetPreset = 0;
  Spark winch;
  AimCommand aim;
  int debounce;
  public HowitzerSystem(JoystickButton in, JoystickButton out, ConveyorSystem s) {
    this.in = in;
    this.out = out;
    aimTalon = new TalonSRX(RobotMap.AIM_TALON.getPin());
    maxLimitSwitch = new DigitalInput(RobotMap.MAX_LIMIT_SWITCH.getPin());
    minLimitSwitch = new DigitalInput(RobotMap.MIN_LIMIT_SWITCH.getPin());

    aimTalon.configContinuousCurrentLimit(40);
    gyro = new PigeonIMU(s.getTalon());
    aim.perpetually().schedule();
  }

  public HowitzerSystem(XboxController operatorController, ConveyorSystem s) {
    this.operatorController = operatorController;
    aimTalon = new TalonSRX(RobotMap.AIM_TALON.getPin());
    maxLimitSwitch = new DigitalInput(RobotMap.MAX_LIMIT_SWITCH.getPin());
    minLimitSwitch = new DigitalInput(RobotMap.MIN_LIMIT_SWITCH.getPin());

    aimTalon.configContinuousCurrentLimit(40);
    gyro = new PigeonIMU(s.getTalon());
  }

  @Override
  public void periodic() {//TODO autoaim toggle instead
    debounce++;
    double[] ypr = new double[3];
    gyro.getYawPitchRoll(ypr);
    howitzerAngle = ypr[2];
    System.out.println(howitzerAngle);
    double error = targetAngle + aimOffset - howitzerAngle;
    if (!maxLimitSwitch.get())
      aimTalon.set(ControlMode.PercentOutput, 0.2);
    else if (!minLimitSwitch.get())
      aimTalon.set(ControlMode.PercentOutput, -.2);
    //else if (operatorController != null && operatorController.getPOV() == 180)
      //aimTalon.set(ControlMode.PercentOutput, -.5);
    //else if (operatorController != null && operatorController.getPOV() == 0)
      //aimTalon.set(ControlMode.PercentOutput, .5);
    else if (error > .1) aimTalon.set(ControlMode.PercentOutput, 
      Constants.howP*error + .12);
    else aimTalon.set(ControlMode.PercentOutput, 0);

    if (operatorController != null && operatorController.getPOV() == 180 && targetPreset > -1 && debounce > 20) {//TODO Better debounce
      debounce = 0;
      targetPreset -= 1;
    }
    else if (operatorController != null && operatorController.getPOV() == 0 && targetPreset < 3 && debounce > 20) {
      debounce = 0;
      targetPreset += 1;
    }

    switch(targetPreset){
      case 0:
        presetAngle = 26;// this one is all the way down, not sure on angle
        break;
      case 1:
        presetAngle = 30.1;
        break;
      case 2:
        presetAngle = 38.5;
        break;
      default:
        break;
    }
    if(targetPreset < 0 || targetPreset > 2) {
      targetAngle = aim.getAngle();
    } else targetAngle = presetAngle;
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