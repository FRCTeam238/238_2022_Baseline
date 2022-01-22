/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.core238.wrappers.SendableWrapper;
import frc.robot.Dashboard238;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Feeder extends Subsystem {
    public final VictorSPX feederMasterDrive = RobotMap.FeederDevices.feederVictor;// FeederDevices.feederTalon;
    public final DigitalInput firstDetector = new DigitalInput(0);
    public final DigitalInput secondDetector = new DigitalInput(1);
    // TODO: change FEEDER_OUTPUT to reasonable value;
    private final double FEEDER_OUTPUT = 1;
    private final double STOP_FEEDER_OUTPUT = 0;
    private int heldBallsNumber = 0;

    private double diagnosticStartTime = 0;

    private boolean lastStateBroken = true;

    Dashboard238 dashboard;

    public Feeder() {
        // initLiveWindow();
        SmartDashboard.putData(this);
        dashboard = Robot.dashboard238;
    }

    @Override
    protected void initDefaultCommand() {
        // TODO Auto-generated method stub
    }

    public void start() {
        feederMasterDrive.set(ControlMode.PercentOutput, FEEDER_OUTPUT);
    }

    public double getPower(){
        double power = feederMasterDrive.getMotorOutputPercent();
        return power;
    }

    public void reverse() {
        feederMasterDrive.set(ControlMode.PercentOutput, -1 * FEEDER_OUTPUT);
    }

    /*
     * if(firstDetector == broken){ turn on }
     * 
     * if(secondDetector == broken){ secondbroken = true } else { if(secondBroken ==
     * true){ turn off secondBroken = false } }
     * 
     * 
     * 
     */

    public void stop() {
        feederMasterDrive.set(ControlMode.PercentOutput, STOP_FEEDER_OUTPUT);
    }

    public void countHeldBalls(){
        if(secondDetector.get() == true && lastStateBroken == false){ // Second sensor is NOT tripped, but just WAS
            heldBallsNumber++;
        }
        lastStateBroken = secondDetector.get();
        SmartDashboard.putNumber("Held Balls", heldBallsNumber - Robot.shooter.ballsShot);
    }
    // public BooleanSupplier getSensor1Triggered (){

    // boolean test = firstDetector.get();

    // BooleanSupplier testBS = new BooleanSupplier(){

    // @Override
    // public boolean getAsBoolean() {
    // // TODO Auto-generated method stub
    // return test;
    // }
    // };
    // return testBS;
    // }
    // public void feederLogicLoop(){
    // boolean lastStateBroken = false;
    // boolean secondSensorBroken = false;
    // boolean firstSensorBroken = false;
    // //firstSensorBroken = firstDetector.get();
    // //secondSensorBroken = secondDetector.get();
    // if(firstSensorBroken == true){
    // start();
    // }
    // if(secondSensorBroken == false && lastStateBroken == true){
    // heldBallsNumber++;
    // stop();
    // }
    // //activate LEDs here
    // if(secondSensorBroken == true){
    // lastStateBroken = true;
    // }
    // }
    // for testing moved to feedercommand

    // private double getMotorOutput(){
    // double motorOutput = feederMasterDrive.getSelectedSensorPosition();
    // return motorOutput;
    // }

    // private void initLiveWindow() {
    // SendableWrapper motor = new SendableWrapper(builder -> {
    // builder.addDoubleProperty("Motor", this::getMotorOutput, null);
    // });

    // addChild("Motor", motor);
    // }

    private List<SendableWrapper> _sendables = new ArrayList<>();

    private void addChild(String name, SendableWrapper wrapper) {
        _sendables.add(wrapper);
        addChild(name, (Sendable) wrapper);
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
            reverse();
        } else if(diagnosticStartTime != 0){
            start();
        }

    }

}
