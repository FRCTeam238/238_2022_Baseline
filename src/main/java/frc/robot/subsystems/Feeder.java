/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.wpilibj.I2C;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.ColorSensorV3.ColorSensorMeasurementRate;
import com.revrobotics.ColorSensorV3.ColorSensorResolution;
import com.revrobotics.ColorSensorV3.GainFactor;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.core238.Logger;
import frc.core238.wrappers.SendableWrapper;
import frc.robot.Dashboard238;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.RobotMap.FeederDevices.FeederDirection;
import frc.robot.commands.ClearIntake;

/**
 * Add your docs here.
 */
public class Feeder extends SubsystemBase {

    //Victor doesnt exist, Spark is the new controller
    // public final VictorSPX feederMasterDrive = RobotMap.TransportDevices.transportVictor;// FeederDevices.feederTalon;

    public final CANSparkMax feederController = RobotMap.FeederDevices.feederController;
    public final DigitalInput firstDetector = new DigitalInput(0);
    public final DigitalInput secondDetector = new DigitalInput(1);
    public final DigitalInput thirdDetector = new DigitalInput(2);
    public final Counter ballCounter = new Counter();
    private FeederDirection prevFeedDirection;
    public int prevBallCount = 0;
    
    private final double FEEDER_OUTPUT = 0.5;
    private final double STOP_FEEDER_OUTPUT = 0;
    private int ballCountOffset = 0;

    private double diagnosticStartTime = 0;

    private boolean lastStateBroken = true;

    Dashboard238 dashboard;

    private SimpleWidget feederSpeedFromDashboard;

    public Feeder() {
        // initLiveWindow();
        // SmartDashboard.putData(this);
        dashboard = Robot.dashboard238;
        prevFeedDirection = FeederDirection.up;
        ballCounter.setDownSource(thirdDetector);
        ballCounter.setUpSource(firstDetector);
        feederController.setIdleMode(IdleMode.kBrake);
        // checkColorReset();
        RobotMap.FeederDevices.ballColor.configureColorSensor(ColorSensorResolution.kColorSensorRes13bit, ColorSensorMeasurementRate.kColorRate25ms, GainFactor.kGain3x);

        feederSpeedFromDashboard = Shuffleboard.getTab("Shooter Tuning").add("Feeder Speed", RobotMap.FeederDevices.highHubUpSpeed);
    }

    public void checkColorReset() {
        if (RobotMap.FeederDevices.ballColor.hasReset()) {
            Logger.Debug("CONFIGURING COLOR SENSOR");
            RobotMap.FeederDevices.ballColor = new ColorSensorV3(I2C.Port.kMXP);
            RobotMap.FeederDevices.ballColor.configureColorSensor(ColorSensorResolution.kColorSensorRes13bit, ColorSensorMeasurementRate.kColorRate25ms, GainFactor.kGain3x);
        /*}else if (RobotMap.FeederDevices.ballColor.getBlue() == 0){
            RobotMap.FeederDevices.ballColor.configureColorSensor(ColorSensorResolution.kColorSensorRes13bit, ColorSensorMeasurementRate.kColorRate25ms, GainFactor.kGain3x);
        */}
    }

    public void up(){
        up(FEEDER_OUTPUT);
    }

    public void up(double upSpeed) {
        
        if (prevFeedDirection == FeederDirection.down) {
            updateBallsHeld();
            ballCounter.setDownSource(thirdDetector);
            ballCounter.setUpSource(firstDetector);
        }

        feederController.set(upSpeed);
        prevFeedDirection = FeederDirection.up;  
        //updatePrevBallsHeld();
    }

    public void down() {
        
        if (prevFeedDirection == FeederDirection.up) {
            updateBallsHeld();
            ballCounter.setDownSource(firstDetector);
            ballCounter.clearUpSource();
            
        } 
        feederController.set(-1 * FEEDER_OUTPUT);
        prevFeedDirection = FeederDirection.down;
        //updatePrevBallsHeld();
    }

    public double getPower(){
        return feederController.get();
    }

    public void stop() {
        feederController.set(STOP_FEEDER_OUTPUT);
    }

    private void updateBallsHeld() {
        ballCountOffset += ballCounter.get();
    }

    public void setCurrentBallsHeld(int ballCount)
    {
        ballCountOffset = ballCount - ballCounter.get();
    }

    public int getCurrentBallsHeld(){
        int count = ballCounter.get();
        if (ballCountOffset + count < 0) {
            resetBallCount();
            return 0;
        }
        // return ballCountOffset + ballCounter.get();
        return ballCountOffset + count;
    }
    public void resetBallCount(){
        ballCounter.reset();
        ballCountOffset = 0;
    }
    public void updatePrevBallsHeld() {
        prevBallCount = getCurrentBallsHeld();
    }
    
    private List<SendableWrapper> _sendables = new ArrayList<>();

    private void addChild(String name, SendableWrapper wrapper) {
        _sendables.add(wrapper);
        addChild(name, (Sendable) wrapper);
    }

    public double getFeederSpeedFromDashboard(){
        double speed;
        if (Robot.shooter.isTuningShooter() == true) {
            speed = feederSpeedFromDashboard.getEntry().getDouble(RobotMap.FeederDevices.highHubUpSpeed);
        } else {
            speed = RobotMap.FeederDevices.highHubUpSpeed;
        }
        return speed;
    }


    public void runFeederDiagnostics(){
        Shuffleboard.selectTab("DiagnosticTab");
        if(diagnosticStartTime == 0){
            diagnosticStartTime = Timer.getFPGATimestamp();
        }
        if((diagnosticStartTime + 2) <= Timer.getFPGATimestamp() && diagnosticStartTime != 0){
            stop();
        }
        if((diagnosticStartTime + 1) <= Timer.getFPGATimestamp() && diagnosticStartTime != 0){
            down();
        } else if(diagnosticStartTime != 0){
            up();
        }

    }

}
