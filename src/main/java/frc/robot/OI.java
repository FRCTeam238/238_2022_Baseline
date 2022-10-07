/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.core238.wrappers.TriggerButton;
import frc.robot.commands.FeederCommand;
import frc.robot.commands.FeederCommandWithColor;
import frc.robot.commands.IntakeExtendCommand;
import frc.robot.commands.IntakeInOutCommand;
import frc.robot.commands.IntakeRetractCommand;
import frc.robot.commands.LowerHubCommand;
import frc.robot.commands.ManualCountReset;
import frc.robot.commands.ManualFeed;
import frc.robot.commands.ManualReverse;
import frc.robot.commands.HighHubCommand;
import frc.robot.commands.RaiseHanger;
import frc.robot.commands.LowerHanger;
import frc.robot.commands.TraversalSequence;
import frc.robot.commands.drivetrainparameters.DriverJoysticks;
import frc.robot.commands.RaiseLowerHanger;
import frc.robot.commands.TankDrive;
import frc.robot.commands.TraversalExtendCommand;
import frc.robot.commands.TraversalRetractCommand;

//TODO: do we need the vision???????????? 
//import frc.robot.commands.VisionDrive;
//import frc.robot.commands.DriveStraightPID;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

    public Joystick leftStick = RobotMap.Joysticks.driverStickLeft;
    public Joystick rightStick = RobotMap.Joysticks.driverStickRight;
    public XboxController operatorController = RobotMap.Joysticks.operatorController;
    // public Drivetrain drivetrain;

    public OI() {

        // extend - up(dpad)
        POVButton intakeExtendButton = new POVButton(operatorController, 0);
        intakeExtendButton.whenPressed(new IntakeExtendCommand());

        // retract - down(dpad)
        POVButton intakeRetractButton = new POVButton(operatorController, 180);
        intakeRetractButton.whenPressed(new IntakeRetractCommand(operatorController));

        // MANUAL shooter, uses right trigger
        TriggerButton highHubShoot = new TriggerButton(operatorController, XboxController.Axis.kRightTrigger.value);
        highHubShoot.whileHeld(new HighHubCommand());

        TriggerButton lowerHubShoot = new TriggerButton(operatorController, XboxController.Axis.kLeftTrigger.value);
        lowerHubShoot.whileHeld((new LowerHubCommand()));
        // uses Y-axis of the right stick to control intake and mecanum wheels
        // (forwards/backward)
        Robot.intake.setDefaultCommand(new IntakeInOutCommand(operatorController, XboxController.Axis.kRightY.value));

        // There is no controller parameter since the feeder is automatic and will
        // relate to shooter
        Robot.feeder.setDefaultCommand(new FeederCommand());
        // Robot.feeder.setDefaultCommand(new FeederCommandWithColor());

        JoystickButton manualFeederForwardButton = new JoystickButton(operatorController,
                XboxController.Button.kLeftBumper.value);
        manualFeederForwardButton.whileHeld(new ManualFeed());

        JoystickButton manualFeederReverseButton = new JoystickButton(operatorController,
                XboxController.Button.kRightBumper.value);
        manualFeederReverseButton.whileHeld(new ManualReverse());

        JoystickButton manualCounterReset = new JoystickButton(operatorController,
                XboxController.Button.kBack.value);
        manualCounterReset.whenPressed(new ManualCountReset());

        JoystickButton extendTraversal = new JoystickButton(operatorController,
                XboxController.Button.kY.value);
        extendTraversal.whenPressed(new TraversalExtendCommand());

        JoystickButton retractTraversal = new JoystickButton(operatorController,
                XboxController.Button.kB.value);
        retractTraversal.whenPressed(new TraversalRetractCommand());

        JoystickButton traversalSequence = new JoystickButton(operatorController,
                XboxController.Button.kX.value);
        traversalSequence.whileHeld(new TraversalSequence());

        CommandScheduler.getInstance().setDefaultCommand(Robot.hanger,
                new RaiseLowerHanger(operatorController, XboxController.Axis.kLeftY.value));

        edu.wpi.first.wpilibj2.command.button.JoystickButton manualHangerSequence = new JoystickButton(
                operatorController,
                XboxController.Button.kA.value);
        manualHangerSequence.whenPressed(new LowerHanger());

        DriverJoysticks myDriverJoysticks = new DriverJoysticks(rightStick, leftStick);
        myDriverJoysticks.invertJoysticks();
        TankDrive tankDriveCommand = new TankDrive(myDriverJoysticks, Robot.drivetrain);
        Robot.drivetrain.setDefaultCommand(tankDriveCommand);
    }
}
