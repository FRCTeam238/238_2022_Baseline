/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

/**
 * Used to map hardware ports and define hardware objects - motor controllers,
 * joysticks, cameras, etc
 */
public final class RobotMap {

  /** Motor controllers for the drive train */
  public static class DrivetrainControllers {
    public static final int DRIVE_TRAIN_RIGHT_CONTROLLER = 14;
    public static final int DRIVE_TRAIN_RIGHT_FOLLOWER = 15;

    public static final int DRIVE_TRAIN_LEFT_CONTROLLER = 0;
    public static final int DRIVE_TRAIN_LEFT_FOLLOWER = 1;

    public static WPI_TalonFX LeftDriveTrainController = new WPI_TalonFX(DRIVE_TRAIN_LEFT_CONTROLLER);
    public static WPI_TalonFX LeftDriveTrainFollower = new WPI_TalonFX(DRIVE_TRAIN_LEFT_FOLLOWER);

    public static WPI_TalonFX RightDriveTrainController = new WPI_TalonFX(DRIVE_TRAIN_RIGHT_CONTROLLER);
    public static WPI_TalonFX RightDriveTrainFollower = new WPI_TalonFX(DRIVE_TRAIN_RIGHT_FOLLOWER);

    public static double deadBandZoneValue = 0.07;//0.05;
    public static double driverSlowSpeedMultiplier = 0.7;
  }

  /** Driver joysticks and operator controllers */
  public static class Joysticks {
    public static int leftStickPort = 2;
    public static int rightStickPort = 1;
    public static int controllerPort = 0;

    public static Joystick driverStickLeft = new Joystick(leftStickPort);
    public static Joystick driverStickRight = new Joystick(rightStickPort);
    public static XboxController operatorController = new XboxController(controllerPort);
  }

  /**
   * Integer settings for the vision subsystem - change camera mode, pipeline, and
   * LED state using these
   */
  public static class LimelightSettings {
    public static int visionMode = 0;
    public static int cameraMode = 1;
    public static int ledsOn = 3;
    public static int ledsBlink = 2;
    public static int ledsOff = 1;
  }

  public static class FeederDevices {
    public static int FEEDER_CONTROLLER = 10;
    public static CANSparkMax feederController = new CANSparkMax(FEEDER_CONTROLLER, MotorType.kBrushless);
    public static ColorSensorV3 ballColor = new ColorSensorV3(I2C.Port.kMXP);// make this work on kMXP or robot
    public static double highHubUpSpeed = 0.65;// 0.7 //0.8; //0.9/1<-original
    public static double lowHubUpSpeed = 0.5;

    public static enum FeederDirection {
      up,
      down
    }
  }

  public static class ShooterDevices {
    public static int SHOOTER_CONTROLLER = 13;
    public static int SHOOTER_FOLLOWER = 12;
    public static int BACKSPIN_CONTROLLER = 5;
    public static CANSparkMax shooterController = new CANSparkMax(SHOOTER_CONTROLLER, MotorType.kBrushless);
    public static CANSparkMax shooterFollower = new CANSparkMax(SHOOTER_FOLLOWER, MotorType.kBrushless);
    public static double SHOOTER_SPEED_TOLERANCE = 200;
    public static double SHOOTER_DEFAULT_HIGH_HUB = 1800;//3078;//3095;//3110; // 3120;//(yellow hex: 4) ||||||| //3195;
                                                         // //ReadingSunday(Yellow Hex: 2)
    public static double SHOOTER_DEFAULT_LOW_HUB = 900;//1520;// 1950; //1900
    public static double SHOOTER_ks = 0.31651;
    public static double SHOOTER_kv = 0.0030083 * 42;

    //backspin??
    public static CANSparkMax BackspinController = new CANSparkMax(BACKSPIN_CONTROLLER, MotorType.kBrushless);
    public static double SHOOTER_DEFAULT_BACKSPIN_HIGH = 2500;
    public static double SHOOTER_DEFAULT_BACKSPIN_LOW = 1650;
    public static double BACKSPIN_ks = 0.31651;
    public static double BACKSPIN_kv = 0.0030083 * 42;

    public static double settlingTime = 0.4;
  }

  public static class IntakeDevices {
    public static int INTAKE_CONTROLLER_ID = 8;
    public static VictorSPX intakeVictor = new VictorSPX(INTAKE_CONTROLLER_ID);
    // TODO: change to real number
    public static int FORWARD_CHANNEL = 6;
    public static int REVERSE_CHANNEL = 1;
    public static DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, FORWARD_CHANNEL,
        REVERSE_CHANNEL);
    public static double intakeSpeed = 1;
    public static double outtakeSpeed = 0.50;

    public static double clearIntakeTime = 0.5;

  }

  // MecanumDevices is a secondary set of wheels after the intake
  public static class MecanumDevices {
    public static int MECANUM_CONTROLLER_ID = 9;
    public static VictorSPX mecanumVictor = new VictorSPX(MECANUM_CONTROLLER_ID);
    public static double mecanumInSpeed = 0.75;
    public static double mecanumOutSpeed = 0.50;
  }

  public static class HangerDevices {
    public static int HANGER_CONTROLLER = 2;
    public static WPI_TalonFX hangerTalon = new WPI_TalonFX(HANGER_CONTROLLER);
    public static DigitalInput downLimitSwitch = new DigitalInput(3);
    public static double controllerDeadzone = 0.2;
    // TODO: these are placeholder values, change them when we know more about the
    // needed speeds
    public static double hangerDownSpeed = 0.5;
    public static double hangerUpSpeed = -0.5;
    public static double upSoftLimitThreshold = -214000;// -204000;
    public static double downSoftLimitThreshold = 0;
    public static int FORWARD_CHANNEL = 4;
    public static int REVERSE_CHANNEL = 3;
    public static DoubleSolenoid traversalSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, FORWARD_CHANNEL,
        REVERSE_CHANNEL);
  }
}
