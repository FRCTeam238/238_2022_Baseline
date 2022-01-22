/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.List;

import frc.robot.Robot;
import frc.robot.commands.BaseCommand;
import frc.robot.commands.IAutonomousCommand;
import frc.robot.subsystems.Drivetrain;

public abstract class BaseTurn extends BaseCommand implements IAutonomousCommand {

  private Drivetrain drivetrain;
  protected double angle = 0;
  private double speed = 0;
  private boolean isAutonomousMode = false;

  public BaseTurn() {
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    requires(Robot.drivetrain);

    drivetrain = Robot.drivetrain;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // todo: add the real turn logic

    double offset = getOffset();
    drivetrain.drive(speed, 0, offset);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    drivetrain.stop();
  }

  @Override
  public void setParameters(List<String> parameters) {
    angle = Double.parseDouble(parameters.get(0));
    speed = Double.parseDouble(parameters.get(1));
  }

  @Override
  public boolean getIsAutonomousMode() {
    return isAutonomousMode;
  }

  @Override
  public void setIsAutonomousMode(boolean isAutonomousMode) {
    this.isAutonomousMode = isAutonomousMode;
  }

  abstract double getOffset();

}
