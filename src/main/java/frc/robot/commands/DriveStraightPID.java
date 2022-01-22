/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.core238.Logger;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;

/** Drives straight using TalonSRX PID - pass in distance in inches */
public class DriveStraightPID extends Command {

  Drivetrain drivetrain;
  Boolean isStarted = false;
  double distance;
  public DriveStraightPID(double neededDistance) {
    requires(Robot.drivetrain);
    drivetrain = Robot.drivetrain;
    distance = neededDistance;
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
    if(!isStarted) {
      Logger.Debug("PID Loop Active");
      drivetrain.driveWithTicks(distance);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    /* TODO: Add a skew to ensure we can correct if we overshoot the target */
    return drivetrain.isAtPosition(distance);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Logger.Debug("PID Loop finished");
    isStarted = false;
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
