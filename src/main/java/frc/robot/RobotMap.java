/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Add your docs here.
 */
public enum RobotMap {
    LEFT_DRIVE_MOTOR_ONE(0), LEFT_DRIVE_MOTOR_TWO(0), RIGHT_DRIVE_MOTOR_ONE(0), RIGHT_DRIVE_MOTOR_TWO(0),

    CLIMB_MOTOR(0),

    AIM_TALON(0),

    BALL_STOPPER_MOTOR(0),

    INTAKE_VICTOR(0),

    CONVEYOR_VICTOR(0), COLOR_VICTOR(0),

    TOP_SHOOTER(0), BOTTOM_SHOOTER(0),

    TOP_ENCODER_A(0), TOP_ENCODER_B(1), BOTTOM_ENCODER_A(2), BOTTOM_ENCODER_B(3),;

    int pin;

    RobotMap(int i) {
        this.pin = i;
    }

    public int getPin() {
        return pin;
    }
}
