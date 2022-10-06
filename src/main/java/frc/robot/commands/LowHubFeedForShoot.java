/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.core238.Logger;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class LowHubFeedForShoot extends CommandBase {
  private double delay;
  public LowHubFeedForShoot(double delayTime) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.delay = delayTime;
    addRequirements(Robot.feeder);
    addRequirements(Robot.shooter);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    double feederSpeedFromDashboard = RobotMap.FeederDevices.lowHubUpSpeed;//Robot.feeder.getFeederSpeedFromDashboard();
    Robot.feeder.up(feederSpeedFromDashboard);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    Robot.feeder.stop();
    Robot.shooter.neutral();
    if (interrupted) {
      Robot.feeder.stop();
      Robot.shooter.neutral();
    }
  }
}
