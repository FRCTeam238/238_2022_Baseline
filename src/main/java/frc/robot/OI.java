/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.POVButton;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.core238.wrappers.TriggerButton;
import frc.robot.commands.FeederCommand;
import frc.robot.commands.IntakeExtendCommand;
import frc.robot.commands.IntakeInOutCommand;
import frc.robot.commands.IntakeRetractCommand;
import frc.robot.commands.LowHubPrepareToShoot;
import frc.robot.commands.LowerHubCommand;
import frc.robot.commands.ManualCountReset;
import frc.robot.commands.ManualFeed;
import frc.robot.commands.ManualReverse;
import frc.robot.commands.ManualShooterCommand;
import frc.robot.commands.SetShooterSpeedCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.ToggleLimelightZoom;
// import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;

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
    POVButton intakeExtendButton = new POVButton(operatorController, 180);
    intakeExtendButton.whenPressed(new IntakeExtendCommand(operatorController));

    // retract - down(dpad)
    POVButton intakeRetractButton = new POVButton(operatorController, 0);
    intakeRetractButton.whenPressed(new IntakeRetractCommand(operatorController));

    // MANUAL shooter, uses right trigger
    TriggerButton manualShoot = new TriggerButton(operatorController, XboxController.Axis.kRightTrigger.value);
    manualShoot.whileHeld(new ManualShooterCommand());

    TriggerButton lowerHubShoot = new TriggerButton(operatorController, XboxController.Axis.kLeftTrigger.value);
    lowerHubShoot.whileHeld((new LowerHubCommand()));
    // uses Y-axis of the right stick to control intake and mecanum wheels
    // (forwards/backward)
    Robot.intake.setDefaultCommand(new IntakeInOutCommand(operatorController, XboxController.Axis.kRightY.value));

    // There is no controller parameter since the feeder is automatic and will
    // relate to shooter
    Robot.feeder.setDefaultCommand(new FeederCommand());

    JoystickButton manualFeederForwardButton = new JoystickButton(operatorController,
        XboxController.Button.kLeftBumper.value);
    manualFeederForwardButton.whileHeld(new ManualFeed());

    JoystickButton manualFeederReverseButton = new JoystickButton(operatorController,
        XboxController.Button.kRightBumper.value);
    manualFeederReverseButton.whileHeld(new ManualReverse());

    JoystickButton manualCounterReset = new JoystickButton(operatorController, 
    XboxController.Button.kBack.value);
    manualCounterReset.whenPressed(new ManualCountReset());

    // JoystickButton climbButton = new JoystickButton(operatorController,
    // XboxController.Button.kY.value);
    // climbButton.whenPressed(new HangCommand());

    // TODO: do we need the vision????????????
    // VisionDrive visionDrive = new VisionDrive();
    // DriveStraightPID driveTenFeetPID = new DriveStraightPID(-48);
    // JoystickButton visionTrackButton = new JoystickButton(leftStick,
    // RobotMap.Buttons.visionTrack); button:1
    // JoystickButton driveTenFeetButton = new JoystickButton(leftStick,
    // RobotMap.Buttons.driveTenFeet); button: 6
    // driveTenFeetButton.whenPressed(driveTenFeetPID);
    // visionTrackButton.whileHeld(visionDrive);

    // Uses right trigger and runs AUTOMATED shooting while held
    // TriggerButton automatedShoot = new TriggerButton(operatorController,
    // XboxController.Axis.kRightTrigger.value);
    // automatedShoot.whileHeld(new ShooterCommand());
  }

  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());
}
