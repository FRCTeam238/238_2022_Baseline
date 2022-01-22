/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.command.Command;
import frc.core238.autonomous.AutonomousModeAnnotation;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
@AutonomousModeAnnotation(parameterNames = {})
public class IntakeInOutCommand extends Command implements IAutonomousCommand {
    private double speed = 0;
    private boolean isDone = false;
    private GenericHID controller;
    private int axis;
    private boolean isAuto = false;
    private final double maxSpeed = 0.5;

    public IntakeInOutCommand(GenericHID controller, int axis) {
        requires(Robot.intake);
        this.controller = controller;
        this.axis = axis;
    }

    public IntakeInOutCommand() {
    }

    @Override
    protected void execute() {
        if (getIsAutonomousMode()) {
            speed = maxSpeed;
        } else {
            speed = controller.getRawAxis(axis) * maxSpeed;
        }
        if (Math.abs(speed) < 0.2) {
            Robot.intake.stop();
        } else {
            Robot.intake.in(speed);
        }
    }

    @Override
    protected void end() {
        Robot.intake.stop();
    }

    @Override
    protected void interrupted() {
        end();
    }

    @Override
    protected boolean isFinished() {
        // TODO Auto-generated method stub
        return isDone;
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
