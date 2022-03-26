/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.Faults;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Hanger;
import frc.core238.Logger;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.RobotMap.HangerDevices;

public class RaiseHanger extends Command {

  // only accept value if x is within low range and if y is outside of small
  // window

  Hanger theHanger = Robot.hanger;

  public RaiseHanger() {
    requires(theHanger);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Logger.Debug("Initialized!");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
      theHanger.raiseLower(RobotMap.HangerDevices.hangerUpSpeed);
      Logger.Debug("executing!");
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    //done extending when top soft limit is reached. Using "faults" API to get soft limit status from Talon FX
    Faults faults = new Faults();
    HangerDevices.hangerTalon.getFaults(faults);
    return faults.ReverseSoftLimit;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    theHanger.brake();
    Logger.Debug("done raising");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    theHanger.brake();
  }
}
