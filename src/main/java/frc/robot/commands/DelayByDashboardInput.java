/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.core238.autonomous.AutonomousModeAnnotation;
import frc.robot.Dashboard238;

@AutonomousModeAnnotation(parameterNames = { "DashboardObjectName"})
public class DelayByDashboardInput extends CommandBase implements IAutonomousCommand {
  private boolean isAutonomousMode = false;
  private double timeToDelay = 0;
  private String dashboardID;
  private double startTime = 0;
  private boolean isDone = false;
  public DelayByDashboardInput() {
    
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    timeToDelay = SmartDashboard.getNumber(dashboardID, 0);
    startTime = Timer.getFPGATimestamp();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    if(Timer.getFPGATimestamp() - startTime >= timeToDelay){
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
    // TODO Auto-generated method stub
    this.isAutonomousMode = isAutonomousMode;
  }

  @Override
  public void setParameters(List<String> parameters) {
    // TODO Auto-generated method stub
    dashboardID = parameters.get(0);
    SmartDashboard.putNumber(dashboardID, timeToDelay);
  }
  
  @Override
  public double getDelay() {
      // TODO Auto-generated method stub
      return 0;
  }
}

