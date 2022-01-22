package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.command.Command;
import frc.core238.Logger;
import frc.robot.subsystems.Shooter;

/**
 * MotorCommand: used to test rpm of the shooter motor
 */
public class MotorCommand extends Command {
    private GenericHID controller;
    private int axis;
    private double necessaryAxis = 0.5;    
    private Shooter shooter;

    public MotorCommand(GenericHID controller, int axis) {
        requires(Robot.shooter);
        this.controller = controller;
        this.axis = axis;
        shooter = Robot.shooter;
    }

    @Override
    protected void execute() {
        double axisPower = controller.getRawAxis(axis);
        if (axisPower >= necessaryAxis) {
            // Logger.Debug("Speed: 4000");
             shooter.setSpeed(4000);
        } else if (axisPower <= -necessaryAxis) {
             shooter.setSpeed(1000);
            //  Logger.Debug("Speed: 1000");
        } else if (axisPower <= necessaryAxis && axisPower >= -necessaryAxis) {
            shooter.setSpeed(0);
            // Logger.Debug("Speed: 0");
        }
    }

    @Override
    protected boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
    }

}