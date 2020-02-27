package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.FindPath;
import frc.robot.subsystems.DriveSystem;

public class TurnCommand extends CommandBase {
    DoubleSupplier getAngle;
    DriveSystem d_system;
    RamseteCommand command;
    public TurnCommand(DriveSystem d_system, DoubleSupplier getAngle) {
        super();
        this.d_system = d_system;
        this.getAngle = new AngleSupplier(getAngle, d_system.getAngOffset());
    }

    @Override
    public void initialize() {
        command = new RamseteCommand(FindPath.getTurn(getAngle.getAsDouble()), d_system::getAngle, new RamseteController(Constants.b, Constants.zeta), new DifferentialDriveKinematics(Constants.trackWidth), d_system::setMotorVelocity, d_system);
        command.schedule();
    }

    @Override
    public void end(boolean bool) {
        command.end(bool);
        super.end(bool);
    }

    public boolean isFinished() {
        return command.isFinished();
    }
}

class AngleSupplier implements DoubleSupplier {
    DoubleSupplier supp;
    double offset;
    AngleSupplier(DoubleSupplier supp, double offset){
        this.supp = supp;
        this.offset = offset;
    }
    @Override
    public double getAsDouble() {
        return supp.getAsDouble() + offset;
    }
}