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
  public DigitalInput forwardLimitSwitch, reverseLimitSwitch;
// instantiation
  HowitzerSystem H_System; 
   //limit switches
  

  
  public HowitzerLimiter(HowitzerSystem H_System) {
    // Use addRequirements() here to declare subsystem dependencies.
  
    addRequirements(H_System);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
        //essentially we want motor to reverse if the limit switch is pressed
        if (forwardLimitSwitch.get()) { // If the forward limit switch is pressed
          H_System.aimTalon.set(ControlMode.Position, -2);} 
      
      if (reverseLimitSwitch.get()) { // If the reversed limit switch is pressed
          H_System.aimTalon.set(ControlMode.Position, 2);}
      } 

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
