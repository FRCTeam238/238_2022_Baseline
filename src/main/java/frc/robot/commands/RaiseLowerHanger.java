/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Intake;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class RaiseLowerHanger extends Command {

  // only accept value if x is within low range and if y is outside of small
  // window

  Hanger theHanger = Robot.hanger;

  double magnitudeY = 0;

  private GenericHID controller;
  private int axisY;

  public RaiseLowerHanger(GenericHID controller, int axisY) {
    requires(theHanger);
    this.controller = controller;
    this.axisY = axisY;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    magnitudeY = controller.getRawAxis(axisY);
    if (magnitudeY >= RobotMap.HangerDevices.controllerDeadzone) {
      theHanger.raiseLower(RobotMap.HangerDevices.hangerUpSpeed);
    } else if (magnitudeY <= -RobotMap.HangerDevices.controllerDeadzone) {
      theHanger.raiseLower(RobotMap.HangerDevices.hangerDownSpeed);
    } else {
      theHanger.brake();
    }

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    theHanger.brake();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {

  }
}
