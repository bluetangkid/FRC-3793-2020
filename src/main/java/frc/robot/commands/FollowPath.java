package frc.robot.commands;

import java.util.ArrayList;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;

public class FollowPath extends RamseteCommand {
    FollowPath(Supplier<Pose2d> pose, BiConsumer<Double, Double> output) {
        super(new Trajectory(new ArrayList<State>()), pose, new RamseteController(Constants.b, Constants.zeta), new DifferentialDriveKinematics(Constants.trackWidth), output, null);
    }
}