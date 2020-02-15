/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TestDriveMotorsCommand extends CommandBase {
  /**
   * Creates a new TestDrveMotorsCommand.
   */
  CANSparkMax motor;
  double speed;
 
  public TestDriveMotorsCommand(CANSparkMax theMotor, double theSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    motor=theMotor;
    speed=theSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    motor.set(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    motor.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}