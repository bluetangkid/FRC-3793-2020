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

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class HowitzerSystem extends SubsystemBase {
  /**
   * Creates a new HowitzerSystem.
   */
  VictorSPX aimVictor;
  TalonSRX aimTalon;

  PIDController howitzerController;
  
  NetworkTable limelightTable;
  NetworkTableEntry horizontalOffset;
  public double xOffset;

  double currentHowitzerPosition;

  final double lengthOfHowitzerIn = 40;
  double howitzerAngle;

  

  public HowitzerSystem() {
    aimVictor = new VictorSPX(RobotMap.AIM_VICTOR.getPin());
    aimTalon = new TalonSRX(RobotMap.AIM_TALON.getPin());
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

    horizontalOffset = limelightTable.getEntry("tx");

  }

  @Override
  public void periodic() {
    xOffset = horizontalOffset.getDouble(0);
    // This method will be called once per scheduler run
  }

 public void goToAngle(double angle){
    double setLength = Math.cos(Math.toRadians(angle))*lengthOfHowitzerIn;
    aimTalon.set(ControlMode.Position, setLength);
  }

  void calculateAngle(){
    howitzerAngle =Math.toDegrees(Math.acos(currentHowitzerPosition/lengthOfHowitzerIn));
  }
}
