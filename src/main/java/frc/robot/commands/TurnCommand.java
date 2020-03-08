package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.FindPath;
import frc.robot.subsystems.DriveSystem;

public class TurnCommand extends CommandBase {
    DoubleSupplier getAngle;
    DriveSystem driveSystem;
    PIDController command;
    public TurnCommand(DriveSystem driveSystem, DoubleSupplier getAngle) {
        super();
        this.driveSystem = driveSystem;
        this.getAngle = new AngleSupplier(getAngle, driveSystem.getAngOffset());
        addRequirements(driveSystem);
    }

    @Override
    public void initialize() {
        command = new PIDController(5.04, 0, 2.47, .001);
        
        //if he too fast just decrease P a little vro
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public void end(boolean bool) {
        command.calculate(measurement, setpoint);
        super.end(bool);
    }

    public boolean isFinished() {
        return command.atSetpoint();
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