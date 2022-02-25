/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.core238.Logger;
import frc.robot.Dashboard238;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Intake extends Subsystem {
    private final VictorSPX intakeMasterDrive = RobotMap.IntakeDevices.intakeVictor;//IntakeDevices.INTAKE_MASTER_TALON;
    private DoubleSolenoid solenoid = RobotMap.IntakeDevices.intakeSolenoid;

    private final VictorSPX mecanumMotor = RobotMap.MecanumDevices.mecanumVictor;

    private final double INTAKEPOWER = 0.5;

    private double diagnosticStartTime = 0;

    private NetworkTableEntry entry;

    Dashboard238 dashboard;

    public Intake() {
        initTalons();
        dashboard = Robot.dashboard238;
        entry = Shuffleboard.getTab("DiagnosticTab").add("IntakeVelocity", 0).getEntry();
        //solenoid = RobotMap.IntakeDevices.intakeSolenoid;
        retractIntake();
    }

    @Override 
    public void initDefaultCommand() {

    }

    public void initTalons() {
        intakeMasterDrive.configFactoryDefault();
        intakeMasterDrive.setInverted(false);
    }

    private void setPower(double speedValue, double mecanumSpeedValue){
        
        
        //if intake is deployed, run the intake
        if (solenoid.get() == DoubleSolenoid.Value.kForward) {
            intakeMasterDrive.set(ControlMode.PercentOutput, speedValue);
            mecanumMotor.set(ControlMode.PercentOutput, mecanumSpeedValue);
        } else {
            //if intake isnt deployed, dont run
            intakeMasterDrive.set(ControlMode.PercentOutput, 0);
            mecanumMotor.set(ControlMode.PercentOutput, 0);
        }
        
        
    }


    private double getPower(){
        double power = intakeMasterDrive.getMotorOutputPercent();
        return power;
    }

    public void in(double speed, double mecanumSpeed) {
        setPower(-speed, -mecanumSpeed);
    }


    public void out(double speed, double mecanumSpeed) {
        setPower(speed, mecanumSpeed);
    }

    public void stop(){
        setPower(0, 0);
    }

    public void extendIntake() {
        solenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void retractIntake() {
        solenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public DoubleSolenoid.Value getDirection() {
        return solenoid.get();
    }

    public BooleanSupplier isOutsideLimits(GenericHID controller, int axis){
        return () -> -0.2 >= controller.getRawAxis(axis) || controller.getRawAxis(axis) >= 0.2;
    }

    public void runIntakeDiagnostics(){
        Shuffleboard.selectTab("DiagnosticTab");
        if(diagnosticStartTime == 0){
            diagnosticStartTime = Timer.getFPGATimestamp();
        }
        if((diagnosticStartTime + 2) <= Timer.getFPGATimestamp() && diagnosticStartTime != 0){
            stop();
            retractIntake();
        }
        if((diagnosticStartTime + 1) <= Timer.getFPGATimestamp() && diagnosticStartTime != 0){
            setPower(-0.5, -0.5); //<-recheck this
            extendIntake();
        } else if(diagnosticStartTime != 0){
            setPower(0.5, 0.5); //<-recheck this
            retractIntake();
        }

    }

}
