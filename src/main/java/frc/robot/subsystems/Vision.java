/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.core238.Logger;
import frc.robot.Dashboard238;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Trig238;

/**
 * Add your docs here.
 */
public class Vision extends Subsystem {

  NetworkTable table;
  NetworkTableEntry tx;
  NetworkTableEntry ty;

  double targetHeight;
  double cameraHeight;
  double heightDifference;

  int closeRangePipeline = 0;
  int longRangePipeline = 9;

  double tolerance = 1;

  boolean isCloseRange = true;

  private double diagnosticStartTime = 0;

  private NetworkTableEntry entryTrackingMode;
  private NetworkTableEntry entryLEDsOn;
  private NetworkTableEntry entryDistance;

  Dashboard238 dashboard;

  public Vision(double targHeight, double camHeight){
    targetHeight = targHeight;
    cameraHeight = camHeight;
    initLimelight();
    dashboard = Robot.dashboard238;
    entryTrackingMode = Shuffleboard.getTab("DiagnosticTab").add("LL Tracking Mode", 0).getEntry();
    entryLEDsOn = Shuffleboard.getTab("DiagnosticTab").add("LL Leds On", 0).getEntry();
    entryDistance = Shuffleboard.getTab("DiagnosticTab").add("LL Distance", 0).getEntry();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public boolean hasTarget(){
    boolean hasTarget = table.getEntry("tv").getDouble(0) == 1;
    return hasTarget;
  }

  /** @return yaw (horizontal angle) read by limelight */
  public double getYaw(){
    // This requires the limelight to be calibrated so that it reads zero while the barrel is lined up with the target
    // Do this from the web interface before competitions
    tx = table.getEntry("tx");
    double yaw = tx.getDouble(0.0);
    return yaw;
  }

  public void cameraMode(){
    table.getEntry("camMode").setNumber(RobotMap.LimelightSettings.cameraMode);
  }
  public void trackingMode(){
    table.getEntry("camMode").setNumber(RobotMap.LimelightSettings.visionMode);
  }
  public void ledsOn(){
    table.getEntry("ledMode").setNumber(RobotMap.LimelightSettings.ledsOn);
  }
  public void ledsOff(){
    table.getEntry("ledMode").setNumber(RobotMap.LimelightSettings.ledsOff);
  }
  public void initLimelight(){
    table = NetworkTableInstance.getDefault().getTable("limelight");
    setPipeline(closeRangePipeline);
    cameraMode();
    ledsOff();
  }

  /** @return pitch (vertical angle) read by limelight */
  public double getPitch(){
    ty = table.getEntry("ty");
    double pitch = ty.getDouble(0.0);
    return pitch;
  }

  /** @return linear distance from target, in inches */
  public double getDistanceToTarget(){
    heightDifference = Math.abs(targetHeight - cameraHeight);
    double distance;
    if(isCloseRange){
      distance = Trig238.calculateDistance(heightDifference, getPitch());
    }else{
      distance = Trig238.calculateDistance(heightDifference, getPitch() + 17);
    }
    return distance;
  }

  public void postValues(){
    //SmartDashboard.putNumber("Limelight Yaw", getYaw());
    //SmartDashboard.putNumber("Limelight Pitch", getPitch());
    // SmartDashboard.putNumber("Limelight Distance to Target", getDistanceToTarget());
  }

  public boolean isWithinRange(){
    boolean inRange = false;
    double currentYaw = getYaw();
    if(hasTarget()){
      inRange = Math.abs(currentYaw) <= tolerance;
      return inRange;
    }else{
      return false;
    }
  }

  public void setPipeline(int pipelineID){
    if(pipelineID == closeRangePipeline){
      isCloseRange = true;
    }else{
      isCloseRange = false;
    }
    table.getEntry("pipeline").setNumber(pipelineID);
  }

  public void swapPipeline(){
    if(isCloseRange){
      setPipeline(longRangePipeline);
    }else{
      setPipeline(closeRangePipeline);
    }
  }

  public void visionDiagnostics(){
    Shuffleboard.selectTab("DiagnosticTab");
    
    entryDistance.setDouble(getDistanceToTarget());
   }
}
