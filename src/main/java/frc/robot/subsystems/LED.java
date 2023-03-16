// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LED extends SubsystemBase {
  
  AddressableLED leftLEDStrip = new AddressableLED(Constants.leftLEDStripID);
  AddressableLED rightLEDStrip = new AddressableLED(Constants.rightLEDStripID);

  AddressableLEDBuffer leftLedBuffer = new AddressableLEDBuffer(32);
  AddressableLEDBuffer rightLedBuffer = new AddressableLEDBuffer(32);

  int hue, saturation, value;
  int r, g, b;
  String color, pattern;
  boolean useHSV;

  public LED() {
    leftLEDStrip.setLength(leftLedBuffer.getLength());
    rightLEDStrip.setLength(rightLedBuffer.getLength());

    hue = 0;
    saturation = 0;
    value = 0;

    r = 0;
    g = 0;
    b = 0;

  }

  public void setStateHSV(int hue, int saturation, int value, String pattern){
      this.hue = hue;
      this.saturation = saturation;
      this.value = value;
      this.pattern = pattern;
      this.useHSV = true;
  }

  public void setStateRGB(int r, int g, int b, String pattern){
      this.r = r;
      this.g = g;
      this.b = b;
      this.pattern = pattern;
      this.useHSV = false;
  }
  private void setLEDStrip(){
      leftLEDStrip.setData(leftLedBuffer);
      rightLEDStrip.setData(rightLedBuffer);
  }

  @Override
  public void periodic() {
      switch(pattern){
          case "Solid":
              for(int i = 0; i < leftLedBuffer.getLength(); i++){
                  if(useHSV){
                      leftLedBuffer.setHSV(i, hue, saturation, value);
                      rightLedBuffer.setHSV(i, hue, saturation, value);
                  }
                  else{
                      leftLedBuffer.setRGB(i, r, g, b);
                      rightLedBuffer.setRGB(i, r, g, b);
                  }       
              } 
          break;
          default:

          break;
      }
      setLEDStrip();
  }
}
