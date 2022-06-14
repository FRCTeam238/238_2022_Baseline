/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.core238.Logger;
import frc.core238.autonomous.AutonomousModeAnnotation;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;

@AutonomousModeAnnotation(parameterNames = {"Timeout"})
public class AutoShooterCommand extends SequentialCommandGroup implements IAutonomousCommand {
  /**
   * Add your docs here.
   */
  Shooter theShooter = Robot.shooter;
  Feeder theFeeder = Robot.feeder;
  boolean isAuto = false;
  double startTime = 0;
  double timeout;
  Timer timer;

  public AutoShooterCommand() {
    addRequirements(theFeeder);
    addRequirements(theShooter);

    addCommands(
      new ManualPrepareToShoot(RobotMap.ShooterDevices.SHOOTER_DEFAULT_HIGH_HUB, RobotMap.ShooterDevices.SHOOTER_DEFAULT_BACKSPIN_HIGH),
      new FeedForShoot());
  }

  @Override
  public void initialize()
  {
    timer.reset();
    timer.start();
  }

  @Override
  public boolean getIsAutonomousMode() {
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
    return (timer.hasElapsed(timeout)); //|| Robot.feeder.getCurrentBallsHeld() == 0);
  }
}
