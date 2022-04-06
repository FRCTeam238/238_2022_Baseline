package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Intake;

public class ClearIntake extends Command {

    public ClearIntake(){
        requires(Robot.intake);
    }
   @Override
   protected void initialize() {
       // TODO Auto-generated method stub
       setInterruptible(false);
       setTimeout(RobotMap.IntakeDevices.clearIntakeTime);
   }

   @Override
   protected void execute() {
       // TODO Auto-generated method stub
       Robot.intake.out(RobotMap.IntakeDevices.outtakeSpeed, RobotMap.MecanumDevices.mecanumOutSpeed);
   }
   
    @Override
    protected boolean isFinished() {
        // TODO Auto-generated method stub
        return isTimedOut();
    }
    
}


