/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Turret;

public class TurnTurretManually extends Command {

  private Turret theTurret = Robot.turret;
  public double leftOperatorJsValue;
  private double turnVelocity = 0;
  private double tuningValue = 0.1;
  private final double turretSpeedScale = 0.35;

  private GenericHID controller;
  private int axis;

  public TurnTurretManually(GenericHID controller, int axis) {
    requires(theTurret);
    this.axis = axis;
    this.controller = controller;
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
    double speed = controller.getRawAxis(axis);
    if (Math.abs(speed) > 0.2){
      turnVelocity = getWantedVelocity(speed);
      theTurret.setTurnVelocity(turnVelocity);
    } else {
      theTurret.stop();
    }
    // turnVelocity = getWantedVelocity();
    // theTurret.setTurnVelocity(turnVelocity);
  }

  protected double getWantedVelocity(double rawSpeed){
    double power = rawSpeed * ((tuningValue * rawSpeed * rawSpeed) + (1-tuningValue));
    return power * turretSpeedScale;
  }
  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    theTurret.neutral();
  }
}
