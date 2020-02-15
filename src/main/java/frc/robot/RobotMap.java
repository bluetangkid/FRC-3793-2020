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

    //front left 32, backleft 36
    //front right 33, back front 32 as of 2/15
    LEFT_DRIVE_MOTOR_ONE(32), LEFT_DRIVE_MOTOR_TWO(36), RIGHT_DRIVE_MOTOR_ONE(33), RIGHT_DRIVE_MOTOR_TWO(31),

    CLIMB_MOTOR(2),

    AIM_TALON(8),

    BALL_STOPPER_MOTOR(0),

    LED_LIGHTS(0),

    BALL_COUNTER(0),

    CONVEYOR_VICTOR(5), COLOR_VICTOR(7), INTAKE_VICTOR(9),

    TOP_SHOOTER(34), BOTTOM_SHOOTER(35),

    TOP_ENCODER_A(0), TOP_ENCODER_B(1), BOTTOM_ENCODER_A(2), BOTTOM_ENCODER_B(3);//might not be using this?

    int pin;

    RobotMap(int i) {
        this.pin = i;
    }

    public int getPin() {
        return pin;
    }
}
