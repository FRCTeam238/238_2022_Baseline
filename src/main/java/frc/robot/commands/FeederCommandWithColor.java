package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.core238.Logger;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;

public class FeederCommandWithColor extends CommandBase {

    public int heldBallsNumber = 0;
    boolean lastStateBroken = true;
    boolean secondSensorBroken = true;
    boolean firstSensorBroken = true;
    boolean hasIllegalBall = false;

    boolean thirdSensorBroken = true;

    Feeder theFeeder = Robot.feeder;

    Shooter theShooter = Robot.shooter;
    LED led = Robot.led;

    public boolean colorMode;

    public FeederCommandWithColor() {
        addRequirements(theFeeder);
        // Use addRequirements() here to declare subsystem dependencies
        // eg. addRequirements(chassis);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        SmartDashboard.putBoolean("Color Sensing?", colorMode);
        led.setColor(1, 150, 0, 0, 0);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        colorMode = SmartDashboard.getBoolean("Color Sensing?", true);

        firstSensorBroken = theFeeder.firstDetector.get();
        secondSensorBroken = theFeeder.secondDetector.get();
        thirdSensorBroken = theFeeder.thirdDetector.get();

        if (thirdSensorBroken == false && hasIllegalBall == false) { // is tripped
            theFeeder.stop();
        } else {
            if (firstSensorBroken == false && secondSensorBroken == true) {
                if (colorMode) {
                    preventWrongColor(); /// check this placement
                    
                }
                if (hasIllegalBall){
                    theFeeder.down();
                    Robot.theClearIntake.schedule();
                }else{
                    theFeeder.up();
                }
                

            } else if (secondSensorBroken == false) {
                if (hasIllegalBall){
                    theFeeder.down();
                    Robot.theClearIntake.start();
                    preventWrongColor();
                    if (DriverStation.getAlliance() == Alliance.Red){
                        theFeeder.setCurrentBallsHeld(0);
                    } else {
                        theFeeder.setCurrentBallsHeld(1);
                    }
                   
                    Logger.Debug("ball count?           ---         " + theFeeder.getCurrentBallsHeld());
                   
                }else{
                    theFeeder.up();
                }
                
            } else {
                theFeeder.stop();
            }
        }

        SmartDashboard.putBoolean("First Sensor", firstSensorBroken);
        SmartDashboard.putBoolean("Second Sensor", secondSensorBroken);
        SmartDashboard.putBoolean("Third Sensor", thirdSensorBroken);
    }

    public String getBallColor() {
        String colorOfBall = "";
        if (RobotMap.FeederDevices.ballColor.isConnected()) {
            if (RobotMap.FeederDevices.ballColor.getBlue() > RobotMap.FeederDevices.ballColor.getRed()) {
                colorOfBall = "blue";
            } else {
                colorOfBall = "red";
            }
        } else {
            colorOfBall = "none";
            Logger.Debug("SENSOR IS NOT CONNECTED");
        }
        SmartDashboard.putString("Ball Color", colorOfBall);
        return colorOfBall;
    }

    public void preventWrongColor() {
        Logger.Debug("CHECKING COLOR");
        if (DriverStation.getAlliance() == Alliance.Red) {
            if (getBallColor() == "blue") {
                Logger.Debug("blue ball!");
                hasIllegalBall = true;
            } else {
                hasIllegalBall = false;
            }
        } else {
            if (getBallColor() == "red") {
                Logger.Debug("red ball!");
                hasIllegalBall = true;
            } else {
                //theFeeder.up();
                hasIllegalBall = false;
            }

        }
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        heldBallsNumber = 0;
        // TODO change hard coded values
        led.setColor(1, 60, 0, 0, 0);
        theFeeder.stop();
    }
}