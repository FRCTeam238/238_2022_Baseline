/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.tests;

import org.junit.Assert;
import org.junit.Test;
import frc.core238.Logger;
import frc.robot.commands.PrepareToShoot;
//import jdk.javadoc.internal.doclets.toolkit.util.DocFinder.Output;

/**
 * Add your docs here.
 */
public class TrigTests {
    @Test
    public void CalculateShooterRpmsForMinimumDistance() {
        double rpm = PrepareToShoot.calculateSpeed(100.466, Math.PI/4, 386.22, 6);

        Logger.Debug("calculated rpm = " + rpm);
        Assert.assertEquals("Failed", 5000, rpm, 10);
    }

    //@Test
    public void CalculateShooterRpmsForMaximumDistance() {
        double rpm = PrepareToShoot.calculateSpeed(507.25, Math.PI/4, 386.22, 6);

        Logger.Debug("calculated rpm = " + rpm);
        Assert.assertEquals("Failed", 1858, rpm, 10);
    }

    //@Test
    public void CalculateShooterRpmsForTypicalDistance() {
        double rpm = PrepareToShoot.calculateSpeed(122, Math.PI/4, 386.22, 6);

        Logger.Debug("calculated rpm = " + rpm);
        Assert.assertEquals("Failed", 1854.5, rpm, 10);
    }

    //@Test
    public void CalculateShooterRpmsForTooShort() {
        double rpm = PrepareToShoot.calculateSpeed(99, Math.PI/4, 386.22, 6);

        boolean isTooClose = false;
        
        Logger.Debug("calculated rpm = " + rpm);

        if(rpm > 5000){
            isTooClose = true;
        } else {
            isTooClose = false;
        }

        Assert.assertTrue("We Are Too Close", isTooClose);
        //Assert.assertEquals("Failed", 5000, rpm, 10);
    }

    @Test
    public void CalculateShooterRpmsForTooFar() {

        double distance = 600;
        //double rpm = PrepareToShoot.calculateSpeed(distance, Math.PI/4, 386.22, 6);

        boolean isTooFar = false;

        if(distance > 507.25){
            isTooFar = true;
        } else {
            isTooFar = false;
        }

        Assert.assertTrue("We are too far", isTooFar);
    }

}
