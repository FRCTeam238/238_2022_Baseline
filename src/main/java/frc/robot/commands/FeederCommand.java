/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.core238.Logger;
import frc.robot.Robot;
import frc.robot.subsystems.Transporter;
import frc.robot.subsystems.LED;

public class FeederCommand extends Command {

  public int heldBallsNumber = 0;
  boolean lastStateBroken = true;
  boolean secondSensorBroken = true;
  boolean firstSensorBroken = true;
  

  Transporter theFeeder = Robot.transporter;
  LED led = Robot.led;

  public FeederCommand() {
    requires(theFeeder);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    led.setColor(1, 150, 0, 0, 0); 
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {   
    feederLogicLoop();    
  }

 public void feederLogicLoop(){
    int beginningNumber = 10;
    int endNumber = 80;
    firstSensorBroken = theFeeder.firstDetector.get();
    
    secondSensorBroken = theFeeder.secondDetector.get();
    
    if(firstSensorBroken == false){ // First sensor IS tripped
      theFeeder.start();       
    }
    if(secondSensorBroken == true && lastStateBroken == false){ // Second sensor is NOT tripped, but just WAS
      heldBallsNumber++;      
      Logger.Debug("Held Balls Count = " + heldBallsNumber);
      endNumber = beginningNumber + (14*heldBallsNumber);
      Color toSet = new Color(238, 238, 0);
      led.setColor(beginningNumber, endNumber, toSet);
      theFeeder.stop();
    }
    //activate LEDs here
    
    lastStateBroken = secondSensorBroken;

    SmartDashboard.putBoolean("First Sensor", firstSensorBroken);
    SmartDashboard.putBoolean("Second Sensor", secondSensorBroken);
}


 
  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    heldBallsNumber = 0;
    //TODO change hard coded values
    led.setColor(1, 60, 0, 0, 0);
    theFeeder.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    theFeeder.stop();
  }
}
