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
import frc.robot.Dashboard238;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Intake extends Subsystem {
    private final VictorSPX intakeMasterDrive = RobotMap.IntakeDevices.intakeVictor;//IntakeDevices.INTAKE_MASTER_TALON;
    private final int forwarChannel = RobotMap.IntakeDevices.FORWARD_CHANNEL;
    private final int reverseChannel = RobotMap.IntakeDevices.REVERSE_CHANNEL;
    private DoubleSolenoid solenoid;

    private final double INTAKEPOWER = 0.5;

    private double diagnosticStartTime = 0;

    private NetworkTableEntry entry;

    Dashboard238 dashboard;

    public Intake() {
        initTalons();
        solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, forwarChannel, reverseChannel);
        dashboard = Robot.dashboard238;
        entry = Shuffleboard.getTab("DiagnosticTab").add("IntakeVelocity", 0).getEntry();
        //solenoid = RobotMap.IntakeDevices.intakeSolenoid;
    }

    @Override 
    public void initDefaultCommand() {

    }

    public void initTalons() {
        intakeMasterDrive.configFactoryDefault();
        intakeMasterDrive.setInverted(true);
    }

    private void setPower(double speedValue){
        intakeMasterDrive.set(ControlMode.PercentOutput, speedValue);
    }

    private double getPower(){
        double power = intakeMasterDrive.getMotorOutputPercent();
        return power;
    }

    public void in(double speed) {
        setPower(speed);
    }

    public void out(double speed) {
        setPower(speed);
    }

    public void stop(){
        setPower(0);
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
            setPower(-0.5);
            extendIntake();
        } else if(diagnosticStartTime != 0){
            setPower(0.5);
            retractIntake();
        }

    }

}
