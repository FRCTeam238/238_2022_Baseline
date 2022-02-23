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

public class ManualFeed extends Command {
  public ManualFeed() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.feeder);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    boolean thirdSensorBroken = Robot.feeder.thirdDetector.get();
    // if the top sensor is broken, and we are not trying to shoot, stop the feeder.
    // Logger.Debug("sensor value = " + thirdSensorBroken);
    if (thirdSensorBroken == false) {
      if (Robot.shooter.isShooting == false) {
        Robot.feeder.stop();
      } else {
        Robot.feeder.up();
      }

    } else {
      Robot.feeder.up();
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
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.feeder.stop();
  }
}
