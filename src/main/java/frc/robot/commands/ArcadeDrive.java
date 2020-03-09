/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
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
  Timer t;
  boolean working;
  int heck;

  public ArcadeDrive(DriveSystem drive, XboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    myDrive = drive;
    this.controller = controller;
    addRequirements(myDrive);
    t = new Timer();
  }

  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    heck++;
    if(controller.getStartButton() && heck > 50) {
      heck = 0;
      working = !working;
    }
    //t.start();
    double turn = -controller.getRawAxis(ControllerMap.leftX);
    double throttle = controller.getTriggerAxis(Hand.kRight) - controller.getTriggerAxis(Hand.kLeft);
    double magnitude = Math.sqrt(turn*turn + throttle*throttle);
    throttle /= magnitude;
    turn /= magnitude;
    if(magnitude < Constants.driveDeadzone) {
        throttle = 0;
        turn = 0;
    } else {
      throttle *= ((Math.max(magnitude, 1) - Constants.driveDeadzone) / (1 - Constants.driveDeadzone));
      turn *= ((Math.max(magnitude, 1) - Constants.driveDeadzone) / (1 - Constants.driveDeadzone));
      throttle *= Constants.throttleMax;
      turn *= Constants.turnMax;
    }
    double leftMotorOutput = -(throttle - turn);
    double rightMotorOutput = throttle + turn;
    System.out.println(leftMotorOutput + "\nR" + rightMotorOutput);
    
    if(leftMotorOutput == 0) {
      myDrive.diss();
    } else {
      //myDrive.getLeftMotorOne().set(leftMotorOutput);
      //myDrive.getRightMotorOne().set(rightMotorOutput);
      //myDrive.setMotorVelocity(leftMotorOutput*Constants.maxVelocity, rightMotorOutput*Constants.maxVelocity); //if drive don't work reduce drive p or remove the 60f and maybe the maxVelocity
      myDrive.setMotorVelocity(leftMotorOutput, rightMotorOutput, working); //if drive don't work reduce drive p or remove the 60f and maybe the maxVelocity
    }
    //t.stop();
    //System.out.println(t.get());
    //t.reset();
    //t.start();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}