package frc.robot.commands.autonomi;

import java.nio.file.Path;

import edu.wpi.first.wpilibj.Filesystem;
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
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;

public class Ideal extends CommandBase {
    Timer timer;
    Trajectory[] trajectories;
    int phase;
    ShootCommand shoot;
    IntakeCommand intake;
    FollowPath follower;
    TurnCommand turn;
    DriveSystem drive;

    public Ideal(ShooterSystem shooter, IntakeSystem intakey, ConveyorSystem conveyey, DriveSystem drive, HowitzerSystem how, boolean offset){
        shoot = new ShootCommand(shooter, conveyey, Constants.shooterSpeedT, Constants.shooterSpeedB);
        intake = new IntakeCommand(intakey, conveyey);
        turn = new TurnCommand(drive, () -> Robot.horizontalOffset.getDouble(0));
        try {
            if(offset)
            follower = new FollowPath(drive, TrajectoryUtil.fromPathweaverJson(Path.of(Filesystem.getDeployDirectory().getPath(), "ideal.wpilib.json")));
            else
            follower = new FollowPath(drive, TrajectoryUtil.fromPathweaverJson(Path.of(Filesystem.getDeployDirectory().getPath(), "ideal_off.wpilib.json")));
        } catch(Exception e){
            e.printStackTrace();
        }
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
                if(timer.hasPeriodPassed(2)) {
                    phase++;
                    timer.stop();
                    timer.reset();
                    timer.start();
                }
                break;
            case(1):
                shoot.execute();
                if(timer.hasPeriodPassed(2)) {
                    phase++;
                    timer.stop();
                    timer.reset();
                    timer.start();
                    shoot.end(false);
                }
                break;
            case(2):
                //follower.execute();
                intake.execute();
                //if(follower.isFinished()) {
                    phase++;
                    intake.end(false);
                    turn.initialize();
                //}
                break;
            case(3):
                turn.execute();
                if(turn.isFinished()) {//TODO lower howitzer for further shot
                    phase++;
                    turn.end(false);
                    timer.start();
                }
                break;
            case(4):
                shoot.execute();
                if(timer.hasPeriodPassed(3)) {
                    phase++;
                    timer.stop();
                    timer.reset();
                    shoot.end(false);
                }
                break;
        }
    }

    public boolean isFinished() {
        return phase > 4;
    }
}