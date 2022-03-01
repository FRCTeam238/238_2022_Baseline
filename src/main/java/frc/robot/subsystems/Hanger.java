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
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.FieldConstants;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Hanger extends Subsystem {
    double hangHeight = FieldConstants.hangHeight;
    double ticksPerInch;
    public Object resetEncoder;
    private final WPI_TalonFX climberTalon = RobotMap.HangerDevices.hangerTalon;

    public Hanger() {
        initTalon();
    }

    @Override
    protected void initDefaultCommand() {
        // TODO Auto-generated method stub
    }

    private void initTalon() {
        climberTalon.configFactoryDefault();
        climberTalon.setNeutralMode(NeutralMode.Brake);
        // double check forward vs reverse
        climberTalon.configForwardSoftLimitThreshold(RobotMap.HangerDevices.upSoftLimitThreshold);
        climberTalon.configReverseSoftLimitThreshold(RobotMap.HangerDevices.downSoftLimitThreshold);
        climberTalon.configForwardSoftLimitEnable(true);
        climberTalon.configReverseSoftLimitEnable(true);
    }

    public void raiseLower(double speed) {
        climberTalon.set(speed);
    }

    public void brake() {
        climberTalon.set(ControlMode.PercentOutput, 0);
    }
}
