/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.HowitzerSystem;

public class AimCommand extends CommandBase {
  /**
   * Creates a new AimCommand.
   */
  HowitzerSystem howitzerSystem;
  DriveSystem driveSystem;

  double calculatedAngle = 28;

  public AimCommand(HowitzerSystem howitzerSystem, DriveSystem driveSystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.howitzerSystem = howitzerSystem;
    this.driveSystem = driveSystem;

    //addRequirements(m_HowitzerSystem, m_DriveSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void execute() {
    double ax, ay;
    double t = 0;
    double Us = 0.007148;
    double Um = -0.0054; //adjust this to match our trajectory
    double dt = .0087;
    double angle;
    double xDist = -0.0254*Robot.zDist.getDoubleArray(new Double[6])[2]; // straight-line distance to the target
    boolean finished = false;
    double Vx, Vy;
    double V = 27; // linear wheelspeed of b a l l
    double x = 0;
    double y = 0;

    angle = (float) Math.toDegrees(Math.sin(2.44 / 7)); //y/x
    Vx = Math.cos(angle * Math.PI / 180) * V + driveSystem.getSpeedAway();
    Vy = Math.sin(angle * Math.PI / 180) * V;
    ax = -(V / .1375) * (Vx * Us + Um * Vy);
    ay = (V / .1375) * (-Vy * Us + Um * Vx);

    while (!finished) {
      if ((y >= 2.44 || Vy > 0) && x < xDist) {
        double Vxn = Vx + dt * ax; // assume constant acceleration during dt
        double Vyn = Vy + dt * ay;
        x = x + dt * (Vx + Vxn) / 2; // trapezoidal integration of position
        y = y + dt * (Vy + Vyn) / 2;
        Vx = Vxn; // update Vx&Vy for next interation
        Vy = Vyn;
        V = Math.sqrt((float)(Vx * Vx + Vy * Vy)); // temporary variable
        ax = -(V / .1375) * (Us * Vx + Um * Vy); // update ax&ay for next iteration:
        ay = -9.81 + (V / .1375) * (Um * Vx - Us * Vy);
        t = t + dt;
      } else {
        //println(y);
        if (Math.abs((float) y - 2.44) < .05) {
          finished = true;
        } else if (y > 2.44) {
          angle -= .2;
        } else angle += .2; // Porportional thing might be faster(PID)
        if(angle > 45) {
          finished = true;
          angle = 45;
        }
        V = 27;
        Vx = Math.cos(angle * Math.PI / 180) * V;
        Vy = Math.sin(angle * Math.PI / 180) * V;
        ax = -(V / .1375) * (Vx * Us + Um * Vy);
        ay = (V / .1375) * (-Vy * Us + Um * Vx);
        t = 0;
        x = 0;
        y = 0;
      }
    }
    if(angle < 45 && angle > 28) calculatedAngle = angle - 45;
    if(xDist == 0) howitzerSystem.goToAngle(-34);
    else howitzerSystem.goToAngle(calculatedAngle); //TODO use pose for dist if there is no ll dist
  }

  public Double[] getLinearDist(){
    Double[] arr = Robot.zDist.getDoubleArray(new Double[3]);
    if(arr[0] == 0) {
      return new Double[3];// TODO jetson stuff
    } else return arr;
  }

  public double getAngle(){
    return calculatedAngle;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}