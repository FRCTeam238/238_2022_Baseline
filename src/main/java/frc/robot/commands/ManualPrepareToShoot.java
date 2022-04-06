/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.core238.Logger;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Shooter;

public class ManualPrepareToShoot extends Command {

  private Shooter theShooter = Robot.shooter;
  private boolean firstIsAtSpeed = true;
  private Timer timer;
  private double settlingTime = RobotMap.ShooterDevices.settlingTime;
  private double rpm;
  private double backspinRpm = 0;

  public ManualPrepareToShoot(double rpm) {
    this.timer = new Timer();
    this.rpm = rpm;
    requires(theShooter);
  }

  public ManualPrepareToShoot(double rpm, double backspinRpm) {
    this.timer = new Timer();
    this.rpm = rpm;
    this.backspinRpm = backspinRpm;
    requires(theShooter);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    timer.reset();
    firstIsAtSpeed = true;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //remove backspin??
    theShooter.setSpeed(rpm, backspinRpm);

    // Logger.Debug("RPM: " + rpm);
    theShooter.isShooting = true;

    if (theShooter.isAtSpeed() && firstIsAtSpeed) {
      firstIsAtSpeed = false;
      timer.start();
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    // Logger.Debug("TIMER: " + timer.get());
    return timer.get() > settlingTime;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    theShooter.isShooting = false;
    // theShooter.neutral();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}