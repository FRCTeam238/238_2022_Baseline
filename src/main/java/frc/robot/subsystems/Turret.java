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
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.core238.wrappers.SendableWrapper;
import frc.robot.Dashboard238;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Turret extends Subsystem {

    //TODO: temporary TalonSRX number for bench testing; CHANGE IT
    private final TalonSRX turretMasterDrive = RobotMap.TurretDevices.turretTalon;
    private final double totalTurretTicks = 44404.0;
    
    public final double limelightAngleConversion = 160;

    final double timeFromNeutralToFull = 0.2; //seconds

    final double kF = 0;
    final double kP = 0.3;
    final double kI = 0.0001;
    final double kD = 0;
    
    private boolean isAtFirst = false;
    private boolean isAtSecond = false;

    private double diagnosticStartTime = 0;
    private NetworkTableEntry entry1;
    private NetworkTableEntry entry2;
    Dashboard238 dashboard;

    public Turret() {
        initTalons();
        resetEncoder();
        // initLiveWindow();
        dashboard = Robot.dashboard238;
        entry1 = Shuffleboard.getTab("DiagnosticTab").add("Turret at First Position", false).getEntry();
        entry2 = Shuffleboard.getTab("DiagnosticTab").add("Turret at Second Position", false).getEntry();
    }

    @Override
    protected void initDefaultCommand() {
        // TODO Auto-generated method stub

    }
    
    public void initTalons() {
        turretMasterDrive.configFactoryDefault();
        turretMasterDrive.setInverted(true);
        turretMasterDrive.selectProfileSlot(0, 0);
        turretMasterDrive.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        turretMasterDrive.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        turretMasterDrive.configClosedloopRamp(timeFromNeutralToFull);
        turretMasterDrive.config_kF(0, kF);
        turretMasterDrive.config_kP(0, kP);
        turretMasterDrive.config_kI(0, kI);
        turretMasterDrive.config_kD(0, kD);
        ///turretMasterDrive.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        //turretMasterDrive.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1, 0);
    }

    private void resetEncoder() {
        turretMasterDrive.setSelectedSensorPosition(0, 0, 0);
    }

    private double getEncoderTicks() {
        double encoderTicks = turretMasterDrive.getSelectedSensorPosition();
        return encoderTicks;
    }

    public void setPosition(double angle) {
        //TODO: convert angle to position(ticks)
        double position = angle;
        turretMasterDrive.set(ControlMode.Position,  position);
    }

    public void setTurnVelocity(double magnitude){
        turretMasterDrive.set(ControlMode.PercentOutput, magnitude);
    }

    public double getPosition() {
        double position = turretMasterDrive.getSelectedSensorPosition();
        return position;
    }

    public void stop(){
        //double currentPosition = getPosition();
        //setPosition(currentPosition);
        turretMasterDrive.set(ControlMode.PercentOutput, 0);
    }

    public void neutral(){
        turretMasterDrive.set(ControlMode.PercentOutput, 0);
    }

    private void initLiveWindow() {
        SendableWrapper power = new SendableWrapper(builder -> {
            builder.addDoubleProperty("Encoder", this::getEncoderTicks, null);
        });

        addChild("Encoder", power);
    }

    private List<SendableWrapper> _sendables = new ArrayList<>();
    private void addChild(String name, SendableWrapper wrapper){
      _sendables.add(wrapper);
      addChild(name, (Sendable)wrapper); 
    }

    public void setVelocity(double velocity){
        turretMasterDrive.set(ControlMode.Velocity, velocity);
    }

    public void runTurretDiagnostics(){
        Shuffleboard.selectTab("DiagnosticTab");
        if(getPosition() >= 4900 && isAtFirst == false){
            isAtFirst = true;
        }
        if(isAtFirst == false){
            setPosition(5000);
        }
        if(isAtFirst && getPosition() >= -4900){
            setPosition(- 5000);
        }
        if(getPosition() <= -4900){
            isAtSecond = true;
            neutral();
        }
        entry1.setBoolean(isAtFirst);
        entry2.setBoolean(isAtSecond);
    }
}
