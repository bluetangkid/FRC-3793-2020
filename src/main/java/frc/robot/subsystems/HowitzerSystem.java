/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class HowitzerSystem extends SubsystemBase {
  /**
   * Creates a new HowitzerSystem.
   */
  VictorSPX aimVictor;
  TalonSRX aimTalon;
  
  NetworkTable limelightTable;
  NetworkTableEntry horizontalOffset;
  public double xOffset;

  

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
}
