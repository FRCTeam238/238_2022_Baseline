/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.Date;
import java.util.List;
import java.util.Locale;

import frc.core238.Logger;
import frc.core238.autonomous.AutonomousModeAnnotation;

@AutonomousModeAnnotation(parameterNames = { "Message", "Level" })
public class LogMessage extends BaseCommand implements IAutonomousCommand {
  // use this to track whether execute has logged a message at least once
  // execute will set this to true
  // end will set this to false
  // isFinished will return this
  // allows for auto mo
  private boolean _finished = false;

  private boolean _isAutoMode = false;
  private String _message;
  private String _level;

  public LogMessage() {
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
    String message = String.format("%tR: %s", new Date(), getMessage());
    switch (getMessage().toLowerCase(Locale.ROOT)) {
    case "trace":
      Logger.Trace(message);
      break;
    case "error":
      Logger.Error(message);
      break;
    case "warn":
      Logger.Warn(message);
      break;
    case "info":
      Logger.Info(message);
      break;
    case "debug":
    default:
      Logger.Debug(message);
      break;
    }

    _finished = true;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return _finished;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    _finished = false;
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }

  @Override
  public boolean getIsAutonomousMode() {
    return _isAutoMode;
  }

  @Override
  public void setIsAutonomousMode(boolean isAutonomousMode) {
    _isAutoMode = isAutonomousMode;
  }

  @Override
  public void setParameters(List<String> parameters) {
    setMessage(parameters.get(0));
    setLevel(parameters.get(1));
  }

  public String getMessage() {
    return _message;
  }

  public void setMessage(String message) {
    _message = message;
  }

  public String getLevel() {
    return _level;
  }

  public void setLevel(String level) {
    _level = level;
  }
}
