package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSystem;

public class FollowPath extends RamseteCommand {
    DriveSystem d_system;
    FollowPath(DriveSystem d_system, Trajectory trajectory) {
        super(trajectory, d_system::getPose, new RamseteController(Constants.b, Constants.zeta), new DifferentialDriveKinematics(Constants.trackWidth), d_system::setMotorVelocity, d_system);
    }
}