/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.core238.Logger;
import frc.core238.autonomous.AutonomousModeAnnotation;
import frc.robot.Dashboard238;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
@AutonomousModeAnnotation(parameterNames = {})
public class IntakeInOutCommand extends Command implements IAutonomousCommand {
    public static boolean isDone = false;
    private GenericHID controller;
    private int axis;
    private boolean isAuto = false;

    // Speed; used to control the intake as well as the mecanum motor values

    private double defaultIntakeSpeed = 0;
    private double defaultMecanumSpeed = 0;

    private Dashboard238 dashboard238 = new Dashboard238();
    private Shuffleboard shuffleboard;
    private NetworkTableEntry intakeEntry;
    private NetworkTableEntry mecanumEntry;
    private NetworkTableEntry deadzoneEntry;
    private double deadzoneDefaultValue = 0;
    private double deadzoneValue = 0.5;
    private boolean intakeDiagnostics;

    public IntakeInOutCommand(GenericHID controller, int axis) {
        requires(Robot.intake);
        this.controller = controller;
        this.axis = axis;
        intakeDiagnostics = Shuffleboard.getTab("DiagnosticTab").add("IntakeDiagnostics", false).withPosition(8, 3).getEntry().getBoolean(false);
        intakeEntry = Shuffleboard.getTab("DiagnosticTab").add("Intake Speed", defaultIntakeSpeed).withPosition(8, 4).getEntry();
        mecanumEntry = Shuffleboard.getTab("DiagnosticTab").add("Mecanum Speed", defaultMecanumSpeed).withPosition(7, 4).getEntry();
        deadzoneEntry = Shuffleboard.getTab("DiagnosticTab").add("Deadzone Value", deadzoneDefaultValue).withPosition(6, 4).getEntry();
    }

    public IntakeInOutCommand() {
    }

    @Override
    protected void execute() {       
        if (intakeDiagnostics) {
            // runIntakeDiagnostics();
        }
        
        if (getIsAutonomousMode()) {
            // intakeSpeed = autoSpeed;
            Robot.intake.in(RobotMap.IntakeDevices.intakeSpeed, RobotMap.MecanumDevices.mecanumInSpeed);
        } else {
            double axisValue = controller.getRawAxis(axis);

            //0.2 is the deadzone in thy controller
            if (Math.abs(axisValue) <= deadzoneValue) {
                Robot.intake.stop();
                
            } else {
                //0.2 is deadzone; if it is greater, then run intake in; else if it is less than -0.2, spit thy balls
                if (axisValue < deadzoneValue) {
                    //Spitting Out
                    Robot.intake.out(RobotMap.IntakeDevices.outtakeSpeed, RobotMap.MecanumDevices.mecanumOutSpeed);
                } else {
                    //Taking In
                    Robot.intake.in(RobotMap.IntakeDevices.intakeSpeed, RobotMap.MecanumDevices.mecanumInSpeed);
                    
                }
            }
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
        return isDone && isAuto;
        //return false;
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
