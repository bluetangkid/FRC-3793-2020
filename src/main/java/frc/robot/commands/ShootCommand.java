/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.revrobotics.ControlType;
import com.revrobotics.EncoderType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ConveyorSystem;
import frc.robot.subsystems.ShooterSystem;
public class ShootCommand extends CommandBase {
  double setpointT, setpointB;
  SimpleMotorFeedforward feedForward;
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
    feedForward = new SimpleMotorFeedforward(Constants.kSShooter, Constants.kVShooter, Constants.kAShooter);
    system.getTopWheel().getPIDController().setP(Constants.kPShooter);
    system.getTopWheel().getPIDController().setI(0);
    system.getTopWheel().getPIDController().setD(0);
    system.getTopWheel().getEncoder(EncoderType.kHallSensor, 42);
    system.getTopWheel().getPIDController().setFeedbackDevice(system.getTopWheel().getEncoder(EncoderType.kHallSensor, 42));

    system.getBottomWheel().getPIDController().setP(Constants.kPShooter);
    system.getBottomWheel().getPIDController().setI(0);
    system.getBottomWheel().getPIDController().setD(0);
    system.getBottomWheel().getPIDController().setFeedbackDevice(system.getBottomWheel().getEncoder(EncoderType.kHallSensor, 42));
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
      conveyor.setVictor(0, false);
    }
    if(phase == 2) {
      system.getBottomWheel().getPIDController().setReference(setpointB*60, ControlType.kVelocity, 0, feedForward.calculate(system.getBottomWheel().getEncoder().getVelocity()/60));
      system.getTopWheel().getPIDController().setReference(setpointT*60, ControlType.kVelocity, 0, feedForward.calculate(system.getTopWheel().getEncoder().getVelocity()/60));
      conveyor.setVictor(Constants.conveyorSpeed, true);
      System.out.println("aa");
    } else if(phase == 1) {
      system.getBottomWheel().getPIDController().setReference(setpointB*60, ControlType.kVelocity, 0, feedForward.calculate(system.getBottomWheel().getEncoder().getVelocity()/60));
      system.getTopWheel().getPIDController().setReference(setpointT*60, ControlType.kVelocity, 0, feedForward.calculate(system.getTopWheel().getEncoder().getVelocity()/60));
      System.out.println("a");
      conveyor.setVictor(0, false);
      if(system.mayShoot()) phase++;
    } else {
      conveyor.setVictor(-1, true);
      if(timer.hasPeriodPassed(.05)) phase++;
    }
    SmartDashboard.putNumber("b", system.getBottomWheel().getEncoder().getVelocity()/60f);
    SmartDashboard.putNumber("t", system.getTopWheel().getEncoder().getVelocity()/60f);
  }

  @Override
  public void end(boolean b) {
    system.getBottomWheel().getPIDController().setReference(0, ControlType.kVelocity);
    system.getTopWheel().getPIDController().setReference(0, ControlType.kVelocity);
    system.getTopWheel().disable();
    system.getBottomWheel().disable();
    conveyor.setVictor(0, false);
  }
}