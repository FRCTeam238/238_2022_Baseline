/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.core238.wrappers;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.buttons.Button;

/**
 * Add your docs here.
 */
public class DPadButton extends Button {
    private final GenericHID joystick;
    private final Direction direction;

    public DPadButton(GenericHID joystick, Direction direction) {
        super();
        this.joystick = joystick;
        this.direction = direction;
    }

    public boolean get() {
        int dPadValue = joystick.getPOV();
        Direction current = Direction.getClosest(dPadValue);
        return current.value == direction.value;
    }

    public static enum Direction {
        Unknown(-1), Up(0), Right(90), Down(180), Left(270);

        private int value;

        private Direction(int direction) {
            this.value = direction;
        }

        public static Direction getClosest(int i){
            if ((i > 315  && i <= 359) || (i >=0 && i <= 45)){
                return Direction.Up;
            }

            if (i > 45 && i <= 135) {
                return Direction.Right;
            }

            if (i > 135 && i <= 225) {
                return Direction.Down;
            }

            if (i > 225 && i <= 315) {
                return Direction.Left;
            }

            return Direction.Unknown;
        }
    }
}
