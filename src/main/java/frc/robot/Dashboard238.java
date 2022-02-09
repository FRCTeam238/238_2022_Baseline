/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drivetrain;

/**
 * Add your docs here.
 */
public class Dashboard238 {

    private HashMap<String, NetworkTableEntry> dashboardEntries = new HashMap<>();
    ShuffleboardTab diagnosticTab;
    SimpleWidget leftEncoder;
    Sendable vision;
    
    public void init() {
        diagnosticTab = Shuffleboard.getTab("DiagnosticTab");

        // create widgets
        buildElement(diagnosticTab, "LEncoder", 0, 1, 1, 1, 0);
        buildElement(diagnosticTab, "REncoder", 0, 1, 1, 1, 1);
        buildElement(diagnosticTab, "P", 0, 1, 1, 3, 0);
        buildElement(diagnosticTab, "I", 0, 1, 1, 3, 1);
        buildElement(diagnosticTab, "D", 0, 1, 1, 3, 2);
        buildElement(diagnosticTab, "Iz", 0, 1, 1, 3, 3);
        buildElement(diagnosticTab, "FF", 0, 1, 1, 4, 0);
        buildElement(diagnosticTab, "MinOutput", 0, 1, 1, 3, 1);
        buildElement(diagnosticTab, "MaxOutput", 0, 1, 1, 3, 2);

        buildElement("IntakeDiagnostics", false, 1, 1, 4, 2);
        // put on tabs
        // create get and set method

    }

    private void buildElement(String elementName, Boolean value, int sizeX, int sizeY, int posX, int posY) {
        SimpleWidget theWidget = diagnosticTab.add(elementName, value);
        dashboardEntries.put(elementName, theWidget.getEntry());
        theWidget.withSize(sizeX, sizeY).withPosition(posX, posY);
    }

    public void buildElement(String elementName, Double value, int sizeX, int sizeY, int posX, int posY) {
        SimpleWidget theWidget = diagnosticTab.add(elementName, value);
        dashboardEntries.put(elementName, theWidget.getEntry());
        theWidget.withSize(sizeX, sizeY).withPosition(posX, posY);
    }

    private void buildElement(String elementName, int value, int sizeX, int sizeY, int posX, int posY) {
        buildElement(diagnosticTab, elementName, value, sizeX, sizeY, posX, posY);
    }

    private void buildElement(ShuffleboardTab tab, String elementName, int value, int sizeX, int sizeY, int posX,
            int posY) {
        SimpleWidget theWidget = tab.add(elementName, value);
        dashboardEntries.put(elementName, theWidget.getEntry());
        theWidget.withSize(sizeX, sizeY).withPosition(posX, posY);
    }

    //public void setLeftEncoderTicks(){
    //    double leftTicks = Robot.drivetrain.getLeftEncoderTicks();
    //    vision.addChild(leftEncoder);
    //}

    public void getPIDvalues(){
        double kP;
        double kI;
        double kD;
        double kIz;
        double kFF;
        double kMinOutput;
        double kMaxOutput;
        
    }
        

}
