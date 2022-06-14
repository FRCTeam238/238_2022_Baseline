/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.core238.wrappers;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Add your docs here.
 */
public class InstantTrigger extends Trigger{

    private BooleanSupplier booleanFunc;

    public InstantTrigger(BooleanSupplier booleanFunc) {
        this.booleanFunc = booleanFunc;
    }

    @Override
    public boolean get() {
        // TODO Auto-generated method stub
        return booleanFunc.getAsBoolean();
    }
}
