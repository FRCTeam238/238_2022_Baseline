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
import frc.robot.RobotMap;

public class LowHubFeedForShoot extends Command {
  private double delay;
  public LowHubFeedForShoot(double delayTime) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.delay = delayTime;
    requires(Robot.feeder);
    requires(Robot.shooter);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(this.timeSinceInitialized() >= delay){
      double feederSpeedFromDashboard = RobotMap.FeederDevices.lowHubUpSpeed;//Robot.feeder.getFeederSpeedFromDashboard();
      Robot.feeder.up(feederSpeedFromDashboard);
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
    Robot.feeder.stop();
    Robot.shooter.neutral();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  //**this is where the command "ends"**
  @Override
  protected void interrupted() {
    Robot.feeder.stop();
    Robot.shooter.neutral();
  }
}
