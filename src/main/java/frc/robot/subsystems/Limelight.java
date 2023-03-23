// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {
  
  private NetworkTable table;
  private NetworkTableEntry tx, ty, tv, ledState, exposureState;
  private NetworkTableEntry botPose;
  private double translationX, translationY, translationZ, roll, pitch, yaw;

  private GenericEntry xEntry, yEntry, vEntry, ledStateEntry, exposureStateEntry, sampleAverageEntry;

  private int pipeline;

  private double[] sampleX = new double[10];

  public Limelight() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    tv = table.getEntry("tv");
    ledState = table.getEntry("ledMode");
    exposureState = table.getEntry("camMode");
    botPose = table.getEntry("botpose");

    xEntry = Constants.limelightDebugTab.add("X Entry", 0).getEntry();
    yEntry = Constants.limelightDebugTab.add("Y Entry", 0).getEntry();
    vEntry = Constants.limelightDebugTab.add("V Entry", 0).getEntry();
    ledStateEntry = Constants.limelightDebugTab.add("LED State Entry", 0).getEntry();
    exposureStateEntry = Constants.limelightDebugTab.add("Exposure State Entry", 0).getEntry();
    sampleAverageEntry = Constants.limelightDebugTab.add("Sample Average Entry", 0).getEntry();


    setLEDOnState(false);
    setLowExposure(false);
    //pipeline = 0;
  }

  public boolean isValidTargetFound(){
      if(tv.getInteger(0) == 0){
          return false;
      }
      else{
          return true;
      }
  }

  public double getX(){
      return tx.getDouble(0.0);
  }

  public double getY(){
      return ty.getDouble(0.0);
  }

  public void setPipeline(int pipeline){
      //this.pipeline = pipeline;
  }

  public void setLEDOnState(boolean state){
      if(state){
        this.ledState.setDouble(3);
      }
      else{
        this.ledState.setDouble(1);
      }
  }

  public void setLowExposure(boolean state){
      if(state){
        this.exposureState.setDouble(0);
      }
      else{
        this.exposureState.setDouble(1);
      }
  }

  public int getPipeline(){
      return this.pipeline;
  }

  //Apriltag and 3D Functions

  public Pose2d getPose(){
      return new Pose2d(translationX, translationY, new Rotation2d(yaw));
  }
  
  public double getSampleAverage(){
    double sum = 0;
    for(int i = 0; i < sampleX.length; i++){
        sum = sum + sampleX[i];
    }
    return sum / sampleX.length;
  }

  @Override
  public void periodic() {

      if(tv.getInteger(0) == 1){
          for(int i = 0; i < sampleX.length; i++){
              sampleX[i] = tx.getDouble(0.0);
          }
      }

      xEntry.setDouble(tx.getDouble(Constants.LIMELIGHT_TX_OFFSET));
      yEntry.setDouble(ty.getDouble(0));
      vEntry.setDouble(tv.getDouble(0));
      ledStateEntry.setInteger(ledState.getInteger(-1));
      exposureStateEntry.setInteger(exposureState.getInteger(-1));
      sampleAverageEntry.setDouble(getSampleAverage());

  }
}
