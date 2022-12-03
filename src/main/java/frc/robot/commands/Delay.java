/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.core238.Logger;
import frc.core238.autonomous.AutonomousModeAnnotation;

@AutonomousModeAnnotation(parameterNames = { "DelayTime" })
public class Delay extends CommandBase implements IAutonomousCommand {
  private double timeToWait = 0;
  private boolean isAutonomousMode = false;
  private double startTime = 0;
  private boolean isDone = false;
  
  public Delay() {
    
  }

  public Delay(double timeToWait) {
    this.timeToWait = timeToWait;
  }

  public void timeout(double time) {
    timeToWait = time;
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    // startTime = Timer.getFPGATimestamp();
    startTime = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    if (startTime == 0) {
      startTime = Timer.getFPGATimestamp();
    }
    double timeProgressed = Timer.getFPGATimestamp() - startTime;
    Logger.Debug("progressedTime: " + timeProgressed);
    if(timeProgressed >= timeToWait){
       isDone = true;
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return isDone;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
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
  @Override
  public double getTimeout() {
      // TODO Auto-generated method stub
      return 0;
  }
}
