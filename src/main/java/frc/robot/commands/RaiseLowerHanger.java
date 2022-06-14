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
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Intake;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class RaiseLowerHanger extends CommandBase {

  // only accept value if x is within low range and if y is outside of small
  // window

  Hanger theHanger = Robot.hanger;

  double magnitudeY = 0;

  private GenericHID controller;
  private int axisY;

  public RaiseLowerHanger(GenericHID controller, int axisY) {
    addRequirements(theHanger);
    this.controller = controller;
    this.axisY = axisY;
    // Use addRequirements() here to declare subsystem dependencies
    // eg. addRequirements(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    magnitudeY = controller.getRawAxis(axisY);
    if (magnitudeY >= RobotMap.HangerDevices.controllerDeadzone) {
      theHanger.raiseLower(RobotMap.HangerDevices.hangerDownSpeed);
    } else if (magnitudeY <= -RobotMap.HangerDevices.controllerDeadzone) {
      theHanger.raiseLower(RobotMap.HangerDevices.hangerUpSpeed);
    } else {
      theHanger.brake();
    }

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    theHanger.brake();
  }
}
