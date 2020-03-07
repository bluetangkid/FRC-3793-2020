/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LightingSystem;

public class anime extends CommandBase {
  /**
   * Creates a new anime.
   */
  LightingSystem lightingSystem;
  int start;
  int end;

  int cur;

  public anime( LightingSystem lightingSystem, int s, int e) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.lightingSystem = lightingSystem;
    start = s;
    end = e;
    cur = start;
    addRequirements(lightingSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    for(int i = start; i<start+end;i++){
      
      if(i != cur){
        lightingSystem.buffer.setHSV(i, 100, 100, 100);
      }else{
        lightingSystem.buffer.setHSV(i, 50, 100, 100);
      }
    }
    lightingSystem.LEDs.setData(lightingSystem.buffer);

    if(cur == end){
      cur = start;
    }else{
      cur ++;
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
