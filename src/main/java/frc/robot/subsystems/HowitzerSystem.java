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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.ControllerMap;
import frc.robot.Robot;
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
  int targetPreset = 2;
  Spark winch;
  AimCommand aim;
  int debounce;
  int debounceT;
  boolean manual = false;
  int LEDMode = 1;
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
    manual = false;
    if(!Robot.auto) {
      switch(operatorController.getPOV()){
        case 180:
          presetAngle = -34;
          break;
        case 90:
          presetAngle = -28.5;
          break;
        case 270:
          presetAngle = -23.5;
          break;
        case 0:
          presetAngle = -1;
          break;
        default:
          manual = true;
          break;
      }
    } else presetAngle = -23.5;
    
    double[] ypr = new double[3];
    gyro.getYawPitchRoll(ypr);
    howitzerAngle = ypr[2] + 50;

    System.out.print("RAW SENSOR ");
    System.out.println(howitzerAngle - 50);
    double error = presetAngle - (howitzerAngle - 50);
    if (!maxLimitSwitch.get())
      aimTalon.set(ControlMode.PercentOutput, 0.3);
    else if (!minLimitSwitch.get())
      aimTalon.set(ControlMode.PercentOutput, -.3);
    else if(manual && Math.abs(operatorController.getRawAxis(ControllerMap.leftY)) > .1) {
      aimTalon.set(ControlMode.PercentOutput, -operatorController.getRawAxis(ControllerMap.leftY));
    } else if (Math.abs(error) > 3 && !manual) aimTalon.set(ControlMode.PercentOutput, Math.signum(error));
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

  public void toggleLimelight(){
    if(LEDMode ==1) LEDMode = 3;
    else LEDMode = 1;

    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(LEDMode);
  }
}