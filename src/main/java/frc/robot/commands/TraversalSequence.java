/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;

public class TraversalSequence extends SequentialCommandGroup {
  /**
   * Add your docs here.
   */


  public TraversalSequence() {
    addCommands(
      new LowerHanger(), //pull onto mid bar
      new TraversalExtendCommand(), //extend traversal to put passive hooks above bar
      new WaitCommand(1.25), //1.34 //1.25          //delay to let passive hooks move. Change from '1' to match needed time
      new RaiseHanger(),         //raise hanger towards high bar
      new TraversalRetractCommand(), //retract traversal to pull extended hook into high bar
      new WaitCommand(1),            //delay to let passive hooks rotate robot. Change time if needed
      new LowerHanger(),           //Pull onto high bar
      new TraversalExtendCommand(), //extend traversal to put passive hooks above high bar
      new WaitCommand(1),          //delay to let passive hooks move. Change from '1' to match needed time
      new RaiseHanger(),         //raise hanger towards traversal bar
      new TraversalRetractCommand(), //retract traversal to pull extended hook into traversal bar
      new WaitCommand(1),            //delay to let passive hooks rotate robot. Change time if needed
      new LowerHanger(),           //Pull onto traversal bar
      new TraversalExtendCommand()); //extend traversal to put passive hooks above traversal bar
  }
}
