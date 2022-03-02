/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.core238.autonomous.AutonomousModeAnnotation;
import frc.robot.Robot;
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
  AutoFeed feedCommand = new AutoFeed(0.5);
  PrepareToShoot prepareToShootCommand = new PrepareToShoot();

  public AutoShooterCommand() {
    requires(theFeeder);
    requires(theShooter);

    
    addParallel(new ManualPrepareToShoot());

    addSequential(new ReadyToShoot());
    
    //This stays the same since it is telling the feeder to run after the shooter
    //is at speed
    addSequential(new AutoFeed(1));

    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.
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
