/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.List;

import frc.core238.Logger;
import frc.core238.autonomous.AutonomousModeAnnotation;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.BaseCommand;
import frc.robot.commands.IAutonomousCommand;
import frc.robot.subsystems.Drivetrain;

@AutonomousModeAnnotation(parameterNames = { "Speed", "Distance", "Intaking" })
public abstract class BaseDriveStraight extends BaseCommand implements IAutonomousCommand {

  private static final double ACCELERATION = 150; // 150 in/sec^2
  private static final double DEL_T = 20.0;

  private double speed = 0;
  // private int distance = 0;
  private double distance = 0;
  private boolean intaking = false;
  private double distanceTravelled = 0;

  private Drivetrain drivetrain;
  private boolean isAutonomousMode = false;
  private boolean started = false;

  private double initialPosL;
  private double initialPosR;
  private boolean backwards = false;
  private double currentVelocity = 0;

  public BaseDriveStraight() {
    requires(Robot.drivetrain);
    requires(Robot.intake);

    drivetrain = Robot.drivetrain;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    drivetrain.resetEncoders();
    initialPosL = drivetrain.leftDistanceTravelled();
    initialPosR = drivetrain.rightDistanceTravelled();
    distanceTravelled = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
      double leftEncoderPos = drivetrain.leftDistanceTravelled();
      double rightEncoderPos = drivetrain.rightDistanceTravelled();

      distanceTravelled = Math.abs(leftEncoderPos - initialPosL + rightEncoderPos - initialPosR) / 2;
      Logger.Debug("DistanceTraveled: " + distanceTravelled);
      if (distanceTravelled >= Math.abs(distance)) {
        drivetrain.stop();
        return;
      }

      double timeToStop = Math.abs(currentVelocity / ACCELERATION);
      double distanceNeededToStop = (Math.abs(currentVelocity) / 2) * timeToStop;

      boolean deAccelerate = (Math.abs(distance) - distanceTravelled <= distanceNeededToStop);

      // // double currentAccel;
      // if (deAccelerate ? !backwards : backwards) {
      //   currentVelocity -= (DEL_T / 1000) * ACCELERATION;
      //   // currentAccel = -ACCELERATION;
      // } else {
      //   currentVelocity += (DEL_T / 1000) * ACCELERATION;
      //   // currentAccel = ACCELERATION;
      // }

      // currentVelocity = Math.max(Math.min(speed, currentVelocity), -speed);

      // // if (Math.abs(speed - Math.abs(currentVelocity)) < 0.5) {
      // // currentAccel = 0;
      // // }

      double offset = getOffset();
      Logger.Debug("Offset: " + offset);
      drivetrain.drive(speed, speed, offset);

      if(intaking) {
        Robot.intake.in(RobotMap.IntakeDevices.intakeSpeed, RobotMap.MecanumDevices.mecanumInSpeed);
      }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return distanceTravelled >= distance;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    drivetrain.stop();
    Robot.intake.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }

  @Override
  public void setParameters(List<String> parameters) {
    speed = Double.parseDouble(parameters.get(0));
    // distance = Integer.parseInt(parameters.get(1));
    distance = Double.parseDouble(parameters.get(1));
    intaking = Boolean.parseBoolean(parameters.get(2));
    backwards = distance < 0;
  }

  protected double getOffset() {
    return 0;
  }

  @Override
  public boolean getIsAutonomousMode() {
    return isAutonomousMode;
  }

  @Override
  public void setIsAutonomousMode(boolean isAutonomousMode) {
    this.isAutonomousMode = isAutonomousMode;
  }

}
