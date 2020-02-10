/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.ShooterSystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ShootCommand extends PIDCommand {
  /**
   * Creates a new ShootCommand.
   */
  private ShooterSystem myShooterSystem;

  public ShootCommand(ShooterSystem sys, CANSparkMax motor, Double setpoint) {
  //TODO Update to spark max
    super(
        // The controller that the command will use
        new PIDController(.1, .0015, .0005),
        // This should return the measurement
        () -> motor.getEncoder().getVelocity(),
        // This should return the setpoint (can also be a constant)
        () -> setpoint,
        // This uses the output
        output -> {
          // Use the output here
          motor.set(output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    myShooterSystem = sys;
    addRequirements(myShooterSystem);
    // Configure additional PID options by calling `getController` here.
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  protected double getSetpoint(){
    return super.m_controller.getSetpoint();
  }

  protected void setSetpoint(double setpoint){
    super.m_controller.setSetpoint(setpoint);
  }
}
