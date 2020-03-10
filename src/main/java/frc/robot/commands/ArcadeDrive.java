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
    double turn = -controller.getRawAxis(ControllerMap.leftX);
    double throttle = controller.getTriggerAxis(Hand.kRight) - controller.getTriggerAxis(Hand.kLeft);
    double magnitude = Math.sqrt(turn*turn + throttle*throttle);
    turn = dz_exp(turn, Constants.driveDeadzone, 3, magnitude);
    throttle = dz_exp(throttle, Constants.driveDeadzone, 3, magnitude);
    double leftMotorOutput = (throttle - turn);
    double rightMotorOutput = throttle + turn;
    System.out.println(leftMotorOutput + "\nR" + rightMotorOutput);
    
    if(leftMotorOutput == 0 && rightMotorOutput == 0) {
      myDrive.diss();
    } else {
      myDrive.setMotorVelocity(leftMotorOutput, rightMotorOutput); //if drive don't work reduce drive p or remove the 60f and maybe the maxVelocity
    }
  }

  double dz_exp(double stick_input, double deadzone, double n, double magnitude){
    double partial_output = dz_scaled_radial(stick_input, deadzone, magnitude);
    if (stick_input == 0) return 0;
    double input_normalized = partial_output / magnitude;
    return input_normalized * Math.pow(magnitude, n);
  }

  double dz_scaled_radial(double stick_input, double deadzone, double magnitude){
	  if (stick_input < deadzone) return 0;
    double normie = stick_input / magnitude;
    return normie * map_range(magnitude, deadzone, 1, 0, 1);
  }

  double map_range(double value, double old_min, double old_max, double new_min, double new_max){
    return (new_min + (new_max - new_min) * (value - old_min) / (old_max - old_min));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}