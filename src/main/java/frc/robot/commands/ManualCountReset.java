package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.core238.autonomous.AutonomousModeAnnotation;
import frc.robot.Robot;

@AutonomousModeAnnotation(parameterNames = {})
public class ManualCountReset extends CommandBase implements IAutonomousCommand{

    public ManualCountReset() {
        
    }
    @Override
    public void execute() {
        Robot.feeder.resetBallCount();
    }
        
    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return true;
    }
    @Override
    public boolean getIsAutonomousMode() {
        // TODO Auto-generated method stub
        return false;
    }
    @Override
    public void setIsAutonomousMode(boolean isAutonomousMode) {
        // TODO Auto-generated method stub
        
    }
    @Override
    public void setParameters(List<String> parameters) {
        // TODO Auto-generated method stub
        
    }
    @Override
    public double getDelay() {
        // TODO Auto-generated method stub
        return 0;
    }
    
}
