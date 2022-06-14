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

public class ManualFeed extends CommandBase {
  public ManualFeed() {
    // Use addRequirements() here to declare subsystem dependencies
    // eg. addRequirements(chassis);
    addRequirements(Robot.feeder);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
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
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    Robot.feeder.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  public void interrupted() {
    Robot.feeder.stop();
  }
}
