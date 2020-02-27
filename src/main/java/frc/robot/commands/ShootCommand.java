/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ConveyorSystem;
import frc.robot.subsystems.ShooterSystem;
public class ShootCommand extends CommandBase {
  double setpointT, setpointB;
  Timer timer;
  int phase;
  ShooterSystem system;
  ConveyorSystem conveyor;
  public ShootCommand(double setpointTop, double setpointBottom, ShooterSystem system, ConveyorSystem conveyor) {
    this.system = system;
    this.setpointT = setpointTop;
    this.setpointB = setpointBottom;
    this.conveyor = conveyor;
    timer = new Timer();
  }

  public void initialize(){
    timer.reset();
    timer.start();
    phase = 0;
    system.getBottomWheel().disable();
    system.getTopWheel().disable();
  }

  public void execute() {
    super.execute();
    if(!system.mayShoot() && phase == 2) {
      phase --;
      conveyor.setVictor(0);
    }
    if(phase == 2) {
      system.setSpeed(setpointT, setpointB);
      conveyor.setVictor(Constants.conveyorSpeed);
    } else if(phase == 1) {
      system.setSpeed(setpointT, setpointB);
      conveyor.setVictor(0);
      if(system.mayShoot()) phase++;
    } else {
      conveyor.setVictor(-1);
      if(timer.hasPeriodPassed(.05)) phase++;
    }
  }

  @Override
  public void end(boolean b) {
    system.setSpeed(0, 0);
    system.getTopWheel().disable();
    system.getBottomWheel().disable();
    conveyor.setVictor(0);
  }
}