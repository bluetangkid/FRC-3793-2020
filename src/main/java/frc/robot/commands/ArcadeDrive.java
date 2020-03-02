/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.revrobotics.ControlType;
import com.revrobotics.EncoderType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.ControllerMap;
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

  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double turn = -controller.getRawAxis(ControllerMap.leftX);
    double throttle = controller.getTriggerAxis(Hand.kRight) - controller.getTriggerAxis(Hand.kLeft);
    double magnitude = Math.max(Math.sqrt(turn*turn + throttle*throttle), 1);
    if(turn < Constants.driveDeadzone || throttle < Constants.driveDeadzone) {
      if(Math.abs(throttle) < Constants.driveDeadzone)
        throttle = 0;
      if(Math.abs(turn) < Constants.driveDeadzone)
      turn = 0;
    } else {
      throttle *= ((magnitude - Constants.driveDeadzone) / (Constants.throttleMax - Constants.driveDeadzone));
      turn *= ((magnitude - Constants.driveDeadzone) / (Constants.turnMax - Constants.driveDeadzone));
      turn = Math.signum(turn)*turn*turn;
    }
    double leftMotorOutput = -(throttle - turn);
    double rightMotorOutput = throttle + turn;
    
    if(leftMotorOutput == 0) {
      myDrive.getLeftMotorOne().set(0);
      myDrive.getRightMotorOne().set(0);
    } else {
      myDrive.setMotorVelocity(leftMotorOutput*60f*Constants.maxVelocity, rightMotorOutput*60f*Constants.maxVelocity);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}