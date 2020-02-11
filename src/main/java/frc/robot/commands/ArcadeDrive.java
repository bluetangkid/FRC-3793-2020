/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSystem;

public class ArcadeDrive extends CommandBase {
  /**
   * Creates a new NormalDrive.
   */
  DriveSystem myDrive;
  XboxController controller;

  public ArcadeDrive(DriveSystem m_Drive, XboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    myDrive = m_Drive;
    this.controller = controller;
    addRequirements(myDrive);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  double dif;
  double leftY = controller.getTriggerAxis(Hand.kRight) - controller.getTriggerAxis(Hand.kLeft);
  double m_leftStick = controller.getX();

  if (Math.abs(leftY) < .05)
    dif = 0.0;
  else {
    dif = (leftY/ Math.abs(leftY))*(.4 + (Math.abs(leftY) * .6));
  }
  double lx = m_leftStick;
  double lNum;
  if (Math.abs(lx) > .25)
    lNum = m_leftStick;
  else
    lNum = 0;
  double leftMotorOutput;
  double rightMotorOutput;

  dif *= -Constants.throttleMax;
  lNum *= Constants.turnMax;

  if (dif >= 0.0) {
      // First quadrant, else second quadrant
      if (lNum >= 0.0) {
          leftMotorOutput = 1;
          rightMotorOutput = dif - lNum;
      } else {
          leftMotorOutput = dif + lNum;
          rightMotorOutput = 1;
      }
  } else {
      // Third quadrant, else fourth quadrant
      if (lNum >= 0.0) {
          leftMotorOutput = dif + lNum;
          rightMotorOutput = 1;
      } else {
          leftMotorOutput = 1;
          rightMotorOutput = dif - lNum;
      }
  }
  //kF is 1/target speed(or 12 because voltage)
  if(lNum == 0 && dif == 0) {
    myDrive.getLeftMotorOne().getPIDController().setReference(0, ControlType.kVelocity);
    myDrive.getRightMotorOne().getPIDController().setReference(0, ControlType.kVelocity);
  } else {
    myDrive.getLeftMotorOne().getPIDController().setReference(leftMotorOutput*Constants.maxVelocity/60f, ControlType.kVelocity);
    myDrive.getRightMotorOne().getPIDController().setReference(rightMotorOutput*Constants.maxVelocity/60f, ControlType.kVelocity);
  }
}

  protected double applyDeadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
        if (value > 0.0) {
            return (value - deadband) / (1.0 - deadband);
        } else {
            return (value + deadband) / (1.0 - deadband);
        }
    } else {
        return 0.0;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
