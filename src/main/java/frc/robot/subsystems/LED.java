/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.util.Color;
import frc.core238.Logger;

/**
 * Add your docs here.
 */
public class LED extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  AddressableLED m_led;
  AddressableLEDBuffer m_ledBuffer;
  int m_dioPort;
  int m_numLeds;
  int m_rainbowFirstPixelHue;

  public LED(final int portId, final int numLeds) {
    this.m_dioPort = portId;
    this.m_numLeds = numLeds;

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    m_led = new AddressableLED(m_dioPort);
    m_ledBuffer = new AddressableLEDBuffer(m_numLeds);

    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();

  }

  /* sets the color of LED id starting at begin thru end */
  public void setColor(final int begin, final int end, final int red, final int green, final int blue) {
    final int howMany = end - begin;

    if( howMany > m_numLeds || end > m_numLeds){
      Logger.Debug("Invalid LED Range for LightShow");
      return;
    }

    for (var i = begin; i < howMany; i++) {
      // Sets the specified LED to the RGB values for Red green blue
      m_ledBuffer.setRGB(i, red, green, blue);
   }
   
   m_led.setData(m_ledBuffer);
   
  }

  /* sets the color of LED id starting at begin thru end */
  public void setColor(final int begin, final int end, Color color) {
    final int howMany = end - begin;
    Logger.Debug("begin = " + begin);
    Logger.Debug("end = " + end);

    if( howMany > m_numLeds || end > m_numLeds){
      Logger.Debug("Invalid LED Range for LightShow");
      return;
    }

    for (var i = begin; i < howMany; i++) {
      // Sets the specified LED to the Color value
      m_ledBuffer.setLED(i, color);
   }
   
   m_led.setData(m_ledBuffer);
   
  }
  
  /* shamelessly taken from wpi docs */
  public void rainbow() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      // Set the value
      m_ledBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
    
    m_led.setData(m_ledBuffer);
    
} 

}
