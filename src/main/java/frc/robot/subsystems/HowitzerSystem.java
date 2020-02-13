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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class HowitzerSystem extends SubsystemBase {
  /**
   * Creates a new HowitzerSystem.
   */
  public TalonSRX aimTalon;

  NetworkTable limelightTable;
  NetworkTableEntry horizontalOffset;
  public double xOffset;

  double currentHowitzerPosition;

  final double lengthOfHowitzerIn = 40;
  public double howitzerAngle;

  public HowitzerSystem() {
    aimTalon = new TalonSRX(RobotMap.AIM_TALON.getPin());
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
    xOffset = horizontalOffset.getDouble(0);
    // This method will be called once per scheduler run
  }

  public void goToAngle(double angle) {
    double setLength = Math.cos(Math.toRadians(angle)) * lengthOfHowitzerIn;
    aimTalon.set(ControlMode.Position, setLength);
  }

  void calculateAngle() {
    howitzerAngle = Math.toDegrees(Math.acos(currentHowitzerPosition / lengthOfHowitzerIn));
  }
}
