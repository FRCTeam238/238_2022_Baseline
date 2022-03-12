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
import frc.robot.FieldConstants;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Trig238;
import frc.robot.subsystems.Shooter;
import frc.core238.Logger;

public class PrepareToShoot extends Command {

  private Shooter theShooter = Robot.shooter;

  private final double gravityAcceleration = 386.22;
  private double shootingAngle = Math.PI / 6; // made-up, IN RADIANS
  private final double wheelRadius = 6;
  private final double defaultSpeed = 4000;
  private static double slipValue = 1;
  private double distance = 150; // -1
  private double firstIsAtSpeedTime = 0;
  private double initialCounterDelay = 1.5;

  public PrepareToShoot() {
    requires(theShooter);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //Logger.Debug("PrepareToShoot Command Executed");
    double wantedSpeed = defaultSpeed;
    boolean shooterHasVision = hasVision();
    if(shooterHasVision){
      distance = getDistanceToTarget();
      wantedSpeed =RobotMap.ShooterDevices.SHOOTER_DEFAULT_HIGH_HUB;
    }
    theShooter.setSpeed(wantedSpeed);
    if(theShooter.isAtSpeed() && firstIsAtSpeedTime == 0){ 
      firstIsAtSpeedTime = this.timeSinceInitialized();
    }
    if((firstIsAtSpeedTime + initialCounterDelay) <= this.timeSinceInitialized()){
      theShooter.beginCounting();
    }
    theShooter.countBalls();
    // SmartDashboard.putNumber("Balls Shot", theShooter.ballsShot);
  }
  // find speed to run at, in ticks per 100ms
  // tell shooter to run at that speed

  // In Velocity mode, output value is in position change / 100ms.
  public static double calculateSpeed(double distance, double shootingAngle, double gravityAcceleration,
      double wheelRadius) {
    double velocityBall;
    double velocityWheel;
    double rotationsPerMinute = 0;

    velocityBall = Trig238.calculateBallVelocity(FieldConstants.VisionConstants.getTargetheight(), gravityAcceleration,
        shootingAngle, distance);

    Logger.Debug("Expected Ball Velocity = " + velocityBall);

    velocityWheel = Trig238.calculateSingleWheelShooterVelocity(velocityBall, wheelRadius,
        FieldConstants.GamePieces.getBallradius());

    calculateSlipValue();

    velocityWheel = slipValue * velocityWheel;

    rotationsPerMinute = 30 * velocityWheel / (wheelRadius * Math.PI);

    if (distance > 507.25 || distance <= 0) {
      rotationsPerMinute = 0;
    }

    if (rotationsPerMinute > 5000) {
      rotationsPerMinute = 5000;
    }

    //Logger.Debug("shooter expected rpm = " + rotationsPerMinute);

    return rotationsPerMinute;
  }

  private double getDistanceToTarget() {

    boolean hasVision = hasVision();
    if (hasVision) {
      distance = Robot.vision.getDistanceToTarget();
    } else {
      distance = 1;
    }
    return distance;
  }

  private static void calculateSlipValue(){
    slipValue = 3;
  }

  private boolean hasVision(){
    boolean hasVision = Robot.vision.hasTarget();
    return hasVision;
  }

  // return distance == -1 ? Robot.vision.getDistanceToTarget() : distance;


  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
    /*
     * double tolerance = 1; // rpm tolerance double shooterSpeed =
     * theShooter.getSpeed(); if (shooterSpeed >= (rotationsPerMinute - tolerance)
     * && shooterSpeed <= (rotationsPerMinute + tolerance)) { return true; } else {
     * return false; }
     */
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