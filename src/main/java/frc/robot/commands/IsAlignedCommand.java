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

public class IsAlignedCommand extends CommandBase {
  public IsAlignedCommand() {
    // Use addRequirements() here to declare subsystem dependencies
    // eg. addRequirements(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    boolean isAligned = Robot.shooter.isAtSpeed() && Robot.vision.isWithinRange();
    return isAligned;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    Logger.Debug("SHOOTER ALIGNED");
  }
}
