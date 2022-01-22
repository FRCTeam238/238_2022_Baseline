/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.nio.channels.Pipe;
import java.util.List;
import java.util.Locale;

import edu.wpi.first.wpilibj.command.Command;
import frc.core238.Logger;
import frc.core238.autonomous.AutonomousModeAnnotation;
import frc.robot.Robot;

@AutonomousModeAnnotation(parameterNames = { "pipeline" })
public class SwapPipelines extends Command implements IAutonomousCommand {

  private int pipeline;

  public SwapPipelines() {
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
    Robot.vision.setPipeline(pipeline);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
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
    return false;
  }

  @Override
  public void setIsAutonomousMode(boolean isAutonomousMode) {
    // TODO Auto-generated method stub

  }

  @Override
  public void setParameters(List<String> parameters) {
    this.pipeline = (int) Double.parseDouble(parameters.get(0));

  }
}
