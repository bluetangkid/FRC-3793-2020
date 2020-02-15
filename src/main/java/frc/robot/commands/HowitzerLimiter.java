/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HowitzerSystem;


public class HowitzerLimiter extends CommandBase {
  // Creates a new HowitzerLimiter.
  DigitalInput forwardLimitSwitch, reverseLimitSwitch;
// instantiation
  HowitzerSystem H_System; 
  DigitalInput forwardLimitSwitchInput, reverseLimitSwitchInput; //limit switches
  

  
  public HowitzerLimiter(HowitzerSystem H_System) {
    // Use addRequirements() here to declare subsystem dependencies.
  
    addRequirements(H_System);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        // limit switches
        
    if (forwardLimitSwitch.get()) { // If the forward limit switch is pressed
        H_System.aimTalon.set(ControlMode.PercentOutput, 0); 
    } 
    else if (reverseLimitSwitch.get()) { // If the reversed limit switch is pressed
        H_System.aimTalon.set(ControlMode.PercentOutput, 0);}
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
