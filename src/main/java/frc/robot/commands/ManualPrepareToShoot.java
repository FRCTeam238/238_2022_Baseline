/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.Shooter;

public class ManualPrepareToShoot extends Command {

  private Shooter theShooter = Robot.shooter;
  private double firstIsAtSpeedTime = 0;
  private double initialCounterDelay = 1.5;

  // TODO: recheck default speed
  private final double defaultSpeed = 4000;

  public ManualPrepareToShoot() {
    requires(theShooter);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double wantedSpeed = defaultSpeed;
    theShooter.setSpeed(wantedSpeed);

    if (theShooter.isAtSpeed() && firstIsAtSpeedTime == 0) {
      firstIsAtSpeedTime = this.timeSinceInitialized();
    }

    if ((firstIsAtSpeedTime + initialCounterDelay) <= this.timeSinceInitialized()) {
      theShooter.beginCounting();
    }

    // TODO: do we need to count balls in the shooter?
    theShooter.countBalls();
    SmartDashboard.putNumber("Balls Shot", theShooter.ballsShot);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    theShooter.neutral();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}