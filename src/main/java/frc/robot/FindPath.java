package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
public class FindPath {
    private static TrajectoryConfig config;

    public static void config(float maxVelocity, float maxAcceleration, float endVelocity, float trackWidth) {
        config = new TrajectoryConfig(maxVelocity, maxAcceleration);
        config.setEndVelocity(endVelocity);
        config.setKinematics(new DifferentialDriveKinematics(trackWidth));
    }
    public static Trajectory generateTrajectory(Pose2d start, Pose2d end, List<Translation2d> interiorPoints) {
        return TrajectoryGenerator.generateTrajectory(start, interiorPoints, end, config);
    }

    // will we give pose robot based or world based? would need to "reset" t265 origin every time we start path for robot based
    public static Trajectory getTurn(double angle) {
        return generateTrajectory(new Pose2d(new Translation2d(), new Rotation2d()), new Pose2d(new Translation2d(), new Rotation2d(angle*Math.PI/180f)), new ArrayList<Translation2d>());
    }

    public static Trajectory getStraight(double dist) {
        return generateTrajectory(new Pose2d(new Translation2d(), new Rotation2d()), new Pose2d(new Translation2d(0, dist), new Rotation2d()), new ArrayList<Translation2d>());
    }
}