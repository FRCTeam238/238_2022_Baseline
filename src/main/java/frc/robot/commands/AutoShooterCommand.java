/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.core238.Logger;
import frc.core238.autonomous.AutonomousModeAnnotation;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;


@AutonomousModeAnnotation(parameterNames = {"Timeout"})
public class AutoShooterCommand extends CommandGroup implements IAutonomousCommand {
  /**
   * Add your docs here.
   */
  Shooter theShooter = Robot.shooter;
  Feeder theFeeder = Robot.feeder;
  boolean isAuto = false;
  double startTime = 0;
  double timeout;
  FeedForShoot feedCommand = new FeedForShoot(0.5);
  PrepareToShoot prepareToShootCommand = new PrepareToShoot();

  public AutoShooterCommand() {
    requires(theFeeder);
    requires(theShooter);

    
    addSequential(new ManualPrepareToShoot(RobotMap.ShooterDevices.SHOOTER_DEFAULT_HIGH_HUB));
   
    //This stays the same since it is telling the feeder to run after the shooter
    //is at speed
    addSequential(new FeedForShoot(0));
  }

  @Override
  public void initialize()
  {
    setTimeout(timeout);
  }

  @Override
  public boolean getIsAutonomousMode() {
    // TODO Auto-generated method stub
    return false;
  }

  @Override
  public void setIsAutonomousMode(boolean isAutonomousMode) {
    this.isAuto = isAutonomousMode;

  }

  @Override
  public void setParameters(List<String> parameters) {
    this.timeout = Double.parseDouble(parameters.get(0));
  }

  @Override
  public boolean isFinished(){
    return (isTimedOut() || Robot.feeder.getCurrentBallsHeld() == 0);
  }
}
