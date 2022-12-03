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
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.core238.Logger;
import frc.core238.autonomous.AutonomousModeAnnotation;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
@AutonomousModeAnnotation(parameterNames = {})
public class TraversalExtendCommand extends CommandBase implements IAutonomousCommand {

    boolean isAuto = false;

    public TraversalExtendCommand() {
    }

    @Override
    public void execute() {
        Robot.hanger.extendTraversal();
    }

    @Override
    public boolean isFinished() {
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
    
    @Override
    public double getDelay() {
        // TODO Auto-generated method stub
        return 0;
    }
}
