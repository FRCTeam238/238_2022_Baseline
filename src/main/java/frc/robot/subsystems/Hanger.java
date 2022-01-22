/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.FieldConstants;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Hanger extends Subsystem {

    // private final TalonSRX hangerMasterDrive =
    // RobotMap.HangerDevices.hangerTalon;
    double hangHeight = FieldConstants.hangHeight;
    double ticksPerInch;
    public Object resetEncoder;
    private final DoubleSolenoid brakeSolenoid = RobotMap.ClimberDevices.brakeSolenoid;
    private final DoubleSolenoid deploySolenoid = RobotMap.ClimberDevices.deploySolenoid;
    private final TalonSRX climberTalon = RobotMap.ClimberDevices.climberTalon;
    final private double yMinimum = 0.5;
    //brakeDelayed is the time of delay we want between stopping the motor and braking. if zero, we assume we can brake immediately after starting motor.
    private final double brakeDelayTime = 0;
    private double timeToEngageBrake = -1;
    public DoubleSolenoid.Value isDeployed = Value.kReverse;

    public Hanger() {
        initTalon();
    }

    @Override
    protected void initDefaultCommand() {
        // TODO Auto-generated method stub

    }

    private void initTalon(){
        climberTalon.configFactoryDefault();
        climberTalon.setNeutralMode(NeutralMode.Brake);
    }

    public void deploy() {

        deploySolenoid.set(DoubleSolenoid.Value.kForward);

    }

    public DoubleSolenoid.Value getBrakeStatus() {
        return brakeSolenoid.get();

    }

    public DoubleSolenoid.Value getDeployStatus() {
        return deploySolenoid.get();
    }

    /*
     * public void toggleBrake(){ DoubleSolenoid.Value brakeStatus =
     * getBrakeStatus(); if(brakeStatus == Value.kForward){
     * brakeSolenoid.set(DoubleSolenoid.Value.kReverse); } else {
     * brakeSolenoid.set(DoubleSolenoid.Value.kForward); } }
     */

    public void brake() {

        climberTalon.set(ControlMode.PercentOutput, 0);
        brakeSolenoid.set(Value.kForward);
    }

    public void unBrake() {

        brakeSolenoid.set(Value.kReverse);
    }

    // raising or lowering the climbing mechanism
    // magnitude is the offset of the joystick used to control raising and lowering
    // the climber
    public void raise(double magnitude) {
        // (tuningValue * (leftJsValue * leftJsValue * leftJsValue) + (1-tuningValue) *
        // leftJsValue);

        if ((Math.abs(magnitude) >= yMinimum) && (isDeployed == Value.kForward)) {

            unBrake();
            timeToEngageBrake = -1;
            climberTalon.set(ControlMode.PercentOutput, magnitude);

        } else {

            if (timeToEngageBrake != -1) {

                if (System.currentTimeMillis() >= timeToEngageBrake) {
                    brake();
                }
            } else {
                timeToEngageBrake = System.currentTimeMillis() + (brakeDelayTime*1000);
            }

        }
    }

}
