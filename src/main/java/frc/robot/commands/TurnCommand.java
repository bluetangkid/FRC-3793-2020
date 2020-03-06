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
    DriveSystem driveSystem;
    RamseteCommand command;
    public TurnCommand(DriveSystem driveSystem, DoubleSupplier getAngle) {
        super();
        this.driveSystem = driveSystem;
        this.getAngle = new AngleSupplier(getAngle, driveSystem.getAngOffset());
        addRequirements(driveSystem);
    }

    @Override
    public void initialize() {
        command = new RamseteCommand(FindPath.getTurn(getAngle.getAsDouble()), driveSystem::getPose, new RamseteController(Constants.b, Constants.zeta), new DifferentialDriveKinematics(Constants.trackWidth), driveSystem::setMotorVelocity, driveSystem);
        command.schedule();//if it moves too slow you gotta go to driveSystem setVelocity and multiply by something idk
        //if he too fast just decrease P a little vro
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