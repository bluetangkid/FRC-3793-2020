/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
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

  public HowitzerSystem() {
    aimTalon = new TalonSRX(RobotMap.AIM_TALON.getPin());
    maxLimitSwitch = new DigitalInput(RobotMap.MAX_LIMIT_SWITCH.getPin());
    minLimitSwitch = new DigitalInput(RobotMap.MIN_LIMIT_SWITCH.getPin());


    aimTalon.configFactoryDefault();
    aimTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    aimTalon.config_kP(0, Constants.kPHow);
    aimTalon.config_kI(0, Constants.kIHow);
    aimTalon.config_kD(0, Constants.kDHow);
    aimTalon.configNominalOutputForward(0, Constants.timeoutMs);
    aimTalon.configNominalOutputReverse(0, Constants.timeoutMs);
    aimTalon.configPeakOutputForward(1, Constants.timeoutMs);
    aimTalon.configPeakOutputReverse(-1, Constants.timeoutMs);
    aimTalon.configAllowableClosedloopError(0, 50);

    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void goToAngle(double angle) {
    double setLength = Math.cos(Math.toRadians(angle + aimOffset)) * lengthOfHowitzerIn;
    aimTalon.set(ControlMode.Position, setLength);
  }

  public void calculateAngle() {
    howitzerAngle = Math.toDegrees(Math.acos(currentHowitzerPosition / lengthOfHowitzerIn));
  }

  public void addOffset(){
    aimOffset += .5;
  }

  public void subOffset(){
    aimOffset -= .5;
  }

  public void limitSwitch(){
    //Values still TBD
    if (!maxLimitSwitch.get()) { // If the forward limit switch is pressed
      aimTalon.set(ControlMode.Position, -2);} 
  
  if (!minLimitSwitch.get()) { // If the reversed limit switch is pressed
      aimTalon.set(ControlMode.Position, 2);}
  } 
  }
}
