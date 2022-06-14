/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.core238.Logger;
import frc.robot.FieldConstants;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * Add your docs here.
 */
public class Hanger extends SubsystemBase {
    double hangHeight = FieldConstants.hangHeight;
    double ticksPerInch;
    public Object resetEncoder;
    private final WPI_TalonFX climberTalon = RobotMap.HangerDevices.hangerTalon;

    public Hanger() {
        initTalon();
    }

    private void initTalon() {
        climberTalon.configFactoryDefault();
        climberTalon.setNeutralMode(NeutralMode.Brake);
        // double check forward vs reverse
        // climberTalon.configForwardSoftLimitThreshold(RobotMap.HangerDevices.upSoftLimitThreshold);
        climberTalon.configReverseSoftLimitThreshold(RobotMap.HangerDevices.upSoftLimitThreshold);
        // climberTalon.configForwardSoftLimitEnable(false);
        climberTalon.configReverseSoftLimitEnable(true);
    }

    public void raiseLower(double speed) {
        if (RobotMap.HangerDevices.downLimitSwitch.get() != false) {
            climberTalon.set(speed);
        } else {
            if (speed < 0) { //check pos or neg
                climberTalon.set(speed);
            } else {
                climberTalon.set(0);
            }
        }
    }

    public void extendTraversal() {
        RobotMap.HangerDevices.traversalSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void retractTraversal() {
        RobotMap.HangerDevices.traversalSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void brake() {
        climberTalon.set(ControlMode.PercentOutput, 0);
    }
}
