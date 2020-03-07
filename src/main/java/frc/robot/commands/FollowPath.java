package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSystem;

public class FollowPath extends RamseteCommand {
    public FollowPath(DriveSystem dSystem, Trajectory trajectory) {
        super(trajectory, dSystem::getPose, new RamseteController(Constants.b, Constants.zeta), new DifferentialDriveKinematics(Constants.trackWidth), dSystem::setMotorVelocity, dSystem);
    }
}