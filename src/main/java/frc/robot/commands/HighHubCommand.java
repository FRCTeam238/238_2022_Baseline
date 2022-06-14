/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.core238.Logger;
import frc.robot.Dashboard238;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;

public class HighHubCommand extends SequentialCommandGroup {
  /**
   * Add your docs here.
   */
  Shooter theShooter = Robot.shooter;
  Feeder theFeeder = Robot.feeder;
  

  public HighHubCommand() {
    double shooterSpeed = theShooter.getHighSpeedFromDashboard();
    addCommands(
      new ManualPrepareToShoot(shooterSpeed, RobotMap.ShooterDevices.SHOOTER_DEFAULT_BACKSPIN_HIGH),
      new FeedForShoot());
  }
}
