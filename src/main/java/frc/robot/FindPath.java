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
import frc.robot.subsystems.DriveSystem;

public class FindPath {
    private static TrajectoryConfig config;

    public static void config(float maxVelocity, float maxAcceleration, float endVelocity, float trackWidth) {
        config = new TrajectoryConfig(maxVelocity, maxAcceleration);
        config.setEndVelocity(endVelocity);
        config.setKinematics(new DifferentialDriveKinematics(trackWidth));
    }

    public static Trajectory generateTrajectory(Pose2d start, Pose2d end, List <Translation2d> interiorPoints) {
        return TrajectoryGenerator.generateTrajectory(start, interiorPoints, end, config);
    }

    // will we give pose robot based or world based? would need to "reset" t265
    // origin every time we start path for robot based
    public static Trajectory getTurn(double angle, DriveSystem drive) {
        ArrayList<Translation2d> interior = new ArrayList<Translation2d>();
        interior.add(new Translation2d());

        return generateTrajectory(drive.getPose(),
            new Pose2d(drive.getPose().getTranslation(), new Rotation2d(angle * Math.PI / 180f)),
            interior);
    }
}