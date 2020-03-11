package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.FindPath;
import frc.robot.subsystems.DriveSystem;

public class GetBall extends CommandBase {
    DriveSystem dSystem;
    RamseteCommand command;
    BallHandler ball;
    public GetBall(DriveSystem dSystem, BallHandler ball) {
        super();
        this.dSystem = dSystem;
        this.ball = ball;
    }

    @Override
    public void initialize() {
        Pose2d pose = dSystem.getPose();
        Double[] goodBall = ball.getCloseBall();
        new FollowPath(dSystem,
            FindPath.generateTrajectory(pose,
                new Pose2d(pose.getTranslation().getX() + Math.cos(goodBall[1]) * goodBall[0], pose.getTranslation().getY() * Math.sin(goodBall[1]) * goodBall[0], pose.getRotation()),
                new ArrayList<Translation2d>()));
        command.schedule();
    }

    @Override
    public void end(boolean bool) {
        command.end(bool);
        super.end(bool);
    }
}