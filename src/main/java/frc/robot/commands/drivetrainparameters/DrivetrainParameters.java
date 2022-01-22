/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrainparameters;

/**
 * Add your docs here.
 */
public class DrivetrainParameters {
    
    public final double Left;
    public final double Right;
    public final double Angle;

    public DrivetrainParameters(double left, double right, double angle) {
        Left = left;
        Right = right;
        Angle = angle;
    }
}
