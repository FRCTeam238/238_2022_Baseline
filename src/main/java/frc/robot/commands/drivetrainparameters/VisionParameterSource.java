/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrainparameters;

import frc.robot.Robot;
import frc.robot.subsystems.Vision;

/**
 * Add your docs here.
 */
public class VisionParameterSource implements IDrivetrainParametersSource {
    private Vision vision;
    double kPAngle = 0.125;
    double kPDistance = 0.5;

    public VisionParameterSource() {
        vision = Robot.vision;
    }

    @Override
    public DrivetrainParameters Get() {
        double angle = vision.getYaw();
        double anglePower = angle * kPAngle;

        double distance = vision.getDistanceToTarget();
        double distancePower = distance * kPDistance;

        return new DrivetrainParameters(anglePower, -anglePower, angle);
    }
}
