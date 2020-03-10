package frc.robot.commands.autonomi;

import java.nio.file.Path;

import com.ctre.phoenix.motorcontrol.ControlMode;

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

public class Bees extends CommandBase {
    Timer timer;
    int phase;
    ShootCommand shoot;
    IntakeSystem intake;
    FollowPath[] follower;
    TurnCommand turn;
    DriveSystem drive;

    public Bees(ShooterSystem shooter, IntakeSystem intakey, ConveyorSystem conveyey, DriveSystem drive, HowitzerSystem how, boolean offset){
        shoot = new ShootCommand(shooter, conveyey, Constants.shooterSpeedT, Constants.shooterSpeedB);
        intake = intakey;
        turn = new TurnCommand(drive, () -> Robot.horizontalOffset.getDouble(0));
        follower = new FollowPath[4];
        try {
            if(offset) follower[0] = new FollowPath(drive, TrajectoryUtil.fromPathweaverJson(Path.of(Filesystem.getDeployDirectory().getPath(), "bees1_offset.wpilib.json")));
            else follower[0] = new FollowPath(drive, TrajectoryUtil.fromPathweaverJson(Path.of(Filesystem.getDeployDirectory().getPath(), "bees1.wpilib.json")));
            follower[1] = new FollowPath(drive, TrajectoryUtil.fromPathweaverJson(Path.of(Filesystem.getDeployDirectory().getPath(), "bees2.wpilib.json")));
            follower[2] = new FollowPath(drive, TrajectoryUtil.fromPathweaverJson(Path.of(Filesystem.getDeployDirectory().getPath(), "bees3.wpilib.json")));
            follower[3] = new FollowPath(drive, TrajectoryUtil.fromPathweaverJson(Path.of(Filesystem.getDeployDirectory().getPath(), "bees4.wpilib.json")));
        } catch(Exception e){
            e.printStackTrace();
        }
        this.drive = drive;
    }

    public void initialize(){
        super.initialize();
        shoot.initialize();
        timer.start();
    }

    public void execute () {
        shoot.spool();
        if(phase == 2 || phase == 5) intake.getIntakeMotor().set(ControlMode.PercentOutput, Constants.intakeSpeed);
        intake.drop();
        switch(phase) {
            case(0)://wait for howitzer to drop
                if(timer.hasPeriodPassed(2)) {
                    phase++;
                    timer.stop();
                    timer.reset();
                    timer.start();
                }
                break;
            case(1)://shoot
                shoot.execute();
                if(timer.hasPeriodPassed(2)) {
                    phase++;
                    timer.stop();
                    timer.reset();
                    timer.start();
                    shoot.end(false);
                }
                break;
            case(2)://go pick up trench balls
                follower[0].execute();
                if(follower[0].isFinished()) {
                    phase++;
                    shoot.initialize();
                    timer.stop();
                    timer.reset();
                }
                break;
            case(3)://drive to the 5 ball shot
                follower[1].execute();
                if(follower[1].isFinished()) {
                    phase++;
                    timer.start();
                }
                break;
            case(4)://shoot 5 ball
                shoot.execute();
                if(timer.hasPeriodPassed(2.5)) {//TODO move howitzer
                    phase++;
                    shoot.end(false);
                    timer.stop();
                    timer.reset();
                }
                break;
            case(5)://move to intake
                follower[2].execute();
                if(follower[2].isFinished()) {
                    phase++;
                }
                break;
            case(6)://move to shoot
                follower[3].execute();
                if(follower[3].isFinished()) {
                    phase++;
                    timer.start();
                }
            case(7)://shoot final 2
                shoot.execute();
                if(timer.hasPeriodPassed(2.5)) {
                    phase++;
                    shoot.end(false);
                    timer.stop();
                }
            default:
                break;
        }
    }

    public boolean isFinished() {
        return phase > 4;
    }
}