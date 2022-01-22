/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.core238.autonomous.AutonomousModeAnnotation;

@AutonomousModeAnnotation(parameterNames = { "DelayTime" })
public class Delay extends Command implements IAutonomousCommand {
  private double timeToWait = 0;
  private boolean isAutonomousMode = false;
  private double startTime = 0;
  private boolean isDone = false;
  public Delay() {
    
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(Timer.getFPGATimestamp() - startTime >= timeToWait){
      isDone = true;
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isDone;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }

  @Override
  public boolean getIsAutonomousMode() {
    // TODO Auto-generated method stub
    return isAutonomousMode;
  }

  @Override
  public void setIsAutonomousMode(boolean isAutonomousMode) {
    this.isAutonomousMode = isAutonomousMode;

  }

  @Override
  public void setParameters(List<String> parameters) {
    // TODO Auto-generated method stub
    timeToWait = Double.parseDouble(parameters.get(0));
  }
}
