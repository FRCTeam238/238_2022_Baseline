/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.core238.wrappers;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * Class used to provide a sendable interface for an arbitrary sendablebuilder.init implementation 
 */
public class SendableWrapper implements Sendable {
    private SendableBuilderInterface builderImpementation;

    /**
     * Creates an instance of the wrapper class using the specified implemenation:
     * e.g. in a subsystem where rightSpeedCotroller is a subsystem member fied of type SendableWrapper
     * 
     * rigthSpeedController = new SendableWrapper(builder -> {
     *  builder.setSmartDashboardType("Speed Controller");
     *  builder.addDoubleProperty("Value", () -> rightMasterDrive.getMotorOutputPercent(), null);
     * });
     *
     * addChild("Left Encoder", leftEncoder);
     * 
     * @param impl
     */
    public SendableWrapper(SendableBuilderInterface impl){
        builderImpementation = impl;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        if (builderImpementation != null){
            builderImpementation.initSendable(builder);
        }
    }
}
