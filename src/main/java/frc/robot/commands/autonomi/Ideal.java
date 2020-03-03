package frc.robot.commands.autonomi;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.FollowPath;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TurnCommand;
import frc.robot.subsystems.ConveyorSystem;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.HowitzerSystem;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.ShooterSystem;

class Ideal extends CommandBase {
    Timer timer;
    Trajectory[] trajectories;
    int phase;
    ShootCommand shoot;
    IntakeCommand intake;
    FollowPath[] followers;
    TurnCommand turn;
    DriveSystem drive;

    Ideal(ShooterSystem shooter, IntakeSystem intakey, ConveyorSystem conveyey, DriveSystem drive, HowitzerSystem how){
        shoot = new ShootCommand(shooter, conveyey, Constants.shooterSpeedT, Constants.shooterSpeedB);
        intake = new IntakeCommand(intakey, conveyey);
        turn = new TurnCommand(drive, () -> Robot.horizontalOffset.getDouble(0));
        followers = new FollowPath[2];
        this.drive = drive;
    }

    public void initialize(){
        super.initialize();
        shoot.initialize();
        intake.initialize();
        timer.start();
    }

    public void execute () {
        switch(phase) {
            case(0):
                if(timer.hasPeriodPassed(2.5)) {
                    phase++;
                    timer.stop();
                    timer.reset();
                    shoot.end(false);
                }
                shoot.execute();
            case(1):
                if(followers[0].isFinished()) {
                    phase++;
                    intake.end(false);
                    turn.initialize();
                }
                followers[0].execute();
                intake.execute();
            case(2):
                if(turn.isFinished()) {
                    phase++;
                    turn.end(false);
                    timer.start();
                }
                turn.execute();
            case(3):
                if(timer.hasPeriodPassed(1.5)) {
                    phase++;
                    timer.stop();
                    timer.reset();
                    shoot.end(false);
                }
                shoot.execute();
        }
    }

    public boolean isFinished() {
        return phase > 3;
    }
}