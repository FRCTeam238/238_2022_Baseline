/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.tests;

import org.junit.Test;
import static org.junit.Assert.assertTrue;

import frc.core238.wrappers.DPadButton.Direction;

/**
 * Add your docs here.
 */
public class DPadButtonTests {
    @Test
    public void dpadTests(){
        for (int i = 0; i <= 360; i++) {
            Direction closest = Direction.getClosest(i);
            assertTrue(((i > 315  && i <= 359) || (i >=0 && i <= 45)) == (closest == Direction.Up));
            assertTrue((i > 45 && i <= 135) == (closest == Direction.Right));
            assertTrue((i > 135 && i <= 225) == (closest == Direction.Down));
            assertTrue((i > 225 && i <= 315) == (closest == Direction.Left));
        }
    }
}
