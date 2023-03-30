// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LED extends SubsystemBase {
  
  AddressableLED leftLEDStrip = new AddressableLED(Constants.leftLEDStripID);

  AddressableLEDBuffer leftLedBuffer = new AddressableLEDBuffer(28);

  int hue, saturation, value;
  int r, g, b;
  String color, pattern;
  boolean useHSV;
  Alliance currentAlliance;

  Timer patternTimer = new Timer();

  public LED() {
    leftLEDStrip.setLength(leftLedBuffer.getLength());

    hue = 255;
    saturation = 255;
    value = 255;

    r = 255;
    g = 255;
    b = 255;

    useHSV = false;

    pattern = "Alliance";

    for(int i = 0; i < leftLedBuffer.getLength(); i++){
        leftLedBuffer.setRGB(i, 255, 255, 255);
    }
    leftLEDStrip.setData(leftLedBuffer);
    leftLEDStrip.start();

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
  }

  @Override
  public void periodic() {
        currentAlliance = DriverStation.getAlliance();
        if(currentAlliance == DriverStation.Alliance.Red){
            setStateRGB(255, 0, 0, "Solid");
        }else if(currentAlliance == DriverStation.Alliance.Blue){
            setStateRGB(0, 0, 255, "Solid");
        }
        else{
            setStateRGB(255, 255, 255, "Solid");
        }


        switch(pattern){
            case "Solid":
                for(int i = 0; i < leftLedBuffer.getLength(); i++){
                    if(useHSV){
                        leftLedBuffer.setHSV(i, hue, saturation, value);
                    }
                    else{
                        leftLedBuffer.setRGB(i, r, g, b);
                    }       
                } 
            break;
            case "Blink":

            break;
            default:

            break;
        }
        setLEDStrip();
  }
}
