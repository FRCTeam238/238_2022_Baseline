/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Command;
import frc.core238.Logger;
import frc.core238.autonomous.AutonomousModeAnnotation;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
@AutonomousModeAnnotation(parameterNames = {})
public class IntakeExtendRetractCommand extends Command implements IAutonomousCommand {

    boolean isAuto = false;
    GenericHID controller;
    int pov;

    public IntakeExtendRetractCommand(GenericHID controller) {
        this.controller = controller;
        requires(Robot.intake);
    }

    @Override
    protected void execute() {
        pov = controller.getPOV(pov);
        Logger.Debug("POV VALUE: " + pov);
        switch (pov) {
            // up
            case 0:
            case 45:
            case 315:
                Robot.intake.extendIntake();
                break;

            // down
            case 180:
            case 225:
            case 135:
                Robot.intake.retractIntake();
                break;

            // nothing is pressed
            default:
                break;
        }
        if (Robot.intake.getDirection() == Value.kReverse) {
            Robot.intake.extendIntake();
        } else {
            Robot.intake.retractIntake();
        }
    }

    @Override
    protected boolean isFinished() {
        // TODO Auto-generated method stub
        return true;
    }

    @Override
    public boolean getIsAutonomousMode() {
        // TODO Auto-generated method stub
        return isAuto;
    }

    @Override
    public void setIsAutonomousMode(boolean isAutonomousMode) {
        // TODO Auto-generated method stub
        isAuto = isAutonomousMode;
    }

    @Override
    public void setParameters(List<String> parameters) {
        // TODO Auto-generated method stub

    }
}
