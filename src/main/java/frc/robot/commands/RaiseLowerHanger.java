/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Hanger;
import frc.robot.Robot;

public class RaiseLowerHanger extends Command {
  
//only accept value if x is within low range and if y is outside of small window

  Hanger theHanger = Robot.hanger;

  double magnitudeX = 0;
  double magnitudeY = 0;
  final private double xMaximum = 0.1;
  final private double tuningValue = 0.2;
  public double leftOperatorJsValue;
  DoubleSolenoid.Value isDeployed = Value.kOff;

  private GenericHID controller;
  private int axisX;
  private int axisY;
  public RaiseLowerHanger(GenericHID controller, int axisX, int axisY) {
    requires(theHanger);
    this.controller = controller;
    this.axisX = axisX;
    this.axisY = axisY;
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
    magnitudeX = controller.getRawAxis(axisX);
    
    magnitudeY = controller.getRawAxis(axisY);
    
    if(Math.abs(magnitudeX) <= xMaximum){

      double power = magnitudeY * ( tuningValue * ( magnitudeY*magnitudeY - 1) + 1);
      theHanger.raise(power);

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
    theHanger.raise(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    theHanger.raise(0);
  }
}
