/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomi;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.DriveSystem;

public class SimpAuto extends CommandBase {
  /**
   * Creates a new SimpleAuto.
   */

   DriveSystem driveSystem;
   ShootCommand shoot;

   Timer t;
   int phase = 0;
  public SimpAuto(DriveSystem d, ShootCommand s) {
    this.shoot = s;
    driveSystem = d;
    t = new Timer();
    addRequirements(d);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    t.stop();
    t.reset();
    t.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(phase) {
      case 0:
        if(t.hasElapsed(2)) {
          phase++;
          t.stop();
          t.reset();
          t.start();
        }
        break;
      case 1:
        shoot.initialize();
        phase++;
        break;
      case 2:
        shoot.execute();
        if(t.hasElapsed(3)) {
          phase++;
          shoot.end(false);
          t.stop();
          t.reset();
          t.start();
        }
        break;
      case 3:
        if(t.hasElapsed(1)){
          driveSystem.setMotorVelocity(0, 0);
        }else{
          driveSystem.setMotorVelocity(-6, 6);
        }
        break;
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
