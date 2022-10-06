package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap;
public class ClearIntake extends CommandBase {

    public ClearIntake(){
        addRequirements(Robot.intake);
    }
   @Override
   public void initialize() {
       // TODO Auto-generated method stub
       withTimeout(RobotMap.IntakeDevices.clearIntakeTime);
   }

   @Override
   public void execute() {
       // TODO Auto-generated method stub
       Robot.intake.out(RobotMap.IntakeDevices.outtakeSpeed, RobotMap.MecanumDevices.mecanumOutSpeed);
   }
   
    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
    }
    
}


