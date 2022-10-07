/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrainparameters;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class DriverJoysticks implements IDrivetrainParametersSource {
    private Joystick left;
    private Joystick right;
    private boolean isInverted = false;

    public DriverJoysticks(Joystick rightStick, Joystick leftStick) {
        this.right = rightStick;
        this.left = leftStick;
    }

    @Override
    public DrivetrainParameters Get() {
        double rightJsValue;
        double leftJsValue;
        if(isInverted){
            leftJsValue = -left.getY();
            rightJsValue = -right.getY();
        }else{
            leftJsValue = left.getY();
            rightJsValue = right.getY();
        }

        //should have some sort of abstraction in case we want to pull number from somewhere else
        double tuningValue = SmartDashboard.getNumber("DRIVETRAIN TUNING", 0.2);

        //scaling the stick to power ratio -- accelate the "power" the closer you get to the high or low position on the sitck 
        //This represents x = ax^3+(1-a)x where leftJsValue = x; tuningValue = a;
        double leftPower = (tuningValue * (leftJsValue * leftJsValue * leftJsValue) + (1-tuningValue) * leftJsValue);
        double rightPower = (tuningValue * (rightJsValue * rightJsValue * rightJsValue) + (1-tuningValue) * rightJsValue);
        return new DrivetrainParameters(leftPower, rightPower, 0);
    }

    public void invertJoysticks(){
        if(isInverted == true){
            isInverted = false;
        }else{
            isInverted = true;
        }
    }
}
