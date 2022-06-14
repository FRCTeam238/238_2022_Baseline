/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.core238.Logger;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;

public class LowerHubCommand extends SequentialCommandGroup {
  /**
   * Add your docs here.
   */
  Shooter theShooter = Robot.shooter;
  Feeder theFeeder = Robot.feeder;


  public LowerHubCommand() {
    double shooterSpeed = theShooter.getLowSpeedFromDashboard();
    addCommands(
      new ManualPrepareToShoot(shooterSpeed, RobotMap.ShooterDevices.SHOOTER_DEFAULT_BACKSPIN_LOW),
      new LowHubFeedForShoot());
  }
}
