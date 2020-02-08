/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSystem;

public class NormalDrive extends CommandBase {
  /**
   * Creates a new NormalDrive.
   */
  private DriveSystem myDrive;
  private DoubleSupplier m_leftStick;
  private DoubleSupplier m_leftTrigger;
  private DoubleSupplier m_rightTrigger;

  public NormalDrive(DriveSystem m_Drive, DoubleSupplier leftStick, DoubleSupplier leftTrigger, DoubleSupplier rightTrigger) {
    // Use addRequirements() here to declare subsystem dependencies.
    myDrive = m_Drive;
    addRequirements(myDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

   double dif;
    
   double leftY =  m_leftTrigger.getAsDouble() - m_rightTrigger.getAsDouble();

  if (Math.abs(leftY) < .05)
    dif = 0.0;
  else {
    dif = (leftY/ Math.abs(leftY))*(.4 + (Math.abs(leftY) * .6));
  }
   double lx = m_leftStick.getAsDouble();
   double lNum;
  if (Math.abs(lx) > .25)
    lNum = m_leftStick.getAsDouble();
  else
    lNum = 0;
  
    if (lNum == 0 && dif == 0){
    myDrive.drive.arcadeDrive(0, 0);
  }else{
    myDrive.drive.arcadeDrive(-dif * 1, lNum * .7);
  }
 
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
