package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;

public class DPad extends Button {

    XboxController joystick;
    Direction direction;

    public DPad(XboxController controller, Direction direction) {
        this.joystick = controller;
        this.direction = direction;
    }

    public static enum Direction {
        UP(0), RIGHT(90), DOWN(180), LEFT(270);

        int direction;

        private Direction(int direction) {
            this.direction = direction;
        }
    }

    public boolean get() {
        int dPadValue = joystick.getPOV();
        return (dPadValue == direction.direction) || (dPadValue == (direction.direction + 45) % 360)
                || (dPadValue == (direction.direction + 315) % 360);
    }

}