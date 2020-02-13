/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.Arrays;
import java.util.Comparator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class BallHandler extends CommandBase {
    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public BallHandler() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Double[] balls = SmartDashboard.getNumberArray("Balls", new Double[0]);
        Double[][] ballSeperated = new Double[balls.length / 2][2];
        for (int i = 0; i < balls.length / 2 - 1; i++) {
            ballSeperated[i] = new Double[] { balls[i * 2], balls[i * 2 + 1] };
        }
        Arrays.sort(ballSeperated, (a, b) -> Double.compare(a[0], b[0]));
        
        // closest would be ballSeperated[0]
    }

    @Override
    public void end(final boolean interrupted) {
        super.end(interrupted);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
