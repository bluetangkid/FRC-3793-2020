package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSystem;

public class TurnCommand extends CommandBase {
    DoubleSupplier getAngle;
    DriveSystem driveSystem;
    PIDController command;
    double offset;
    public TurnCommand(DriveSystem driveSystem, DoubleSupplier getAngle) {//TODO turn bias
        super();
        this.driveSystem = driveSystem;
        this.getAngle = new AngleSupplier(getAngle, driveSystem.getAngOffset());
        addRequirements(driveSystem);
    }

    @Override
    public void initialize() {
        command = new PIDController(5.04, 0, 2.47, .001);
        offset = driveSystem.getAngle();
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public void end(boolean bool) {
        super.end(bool);
    }

    public boolean isFinished() {
        return command.atSetpoint();
    }

    public double getBias(){
        return command.calculate(driveSystem.getAngle(), Robot.horizontalOffset.getDouble(0) + offset);
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