// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Grabber extends SubsystemBase {
  /** Creates a new Grabber. */

  private CANSparkMax grabberMotor = new CANSparkMax(Constants.grabberID, MotorType.kBrushless);
  private SparkMaxPIDController m_pidController;
  private RelativeEncoder m_encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  public double rotations = 0;

  GenericEntry pGainEntry = Constants.grabberDebugTab.add("P Gain", 0).getEntry();
  GenericEntry iGainEntry = Constants.grabberDebugTab.add("I Gain", 0).getEntry();
  GenericEntry dGainEntry = Constants.grabberDebugTab.add("D Gain", 0).getEntry();
  GenericEntry iZGainEntry = Constants.grabberDebugTab.add("I Zone", 0).getEntry();
  GenericEntry maxOGainEntry = Constants.grabberDebugTab.add("Max Output", 0).getEntry();
  GenericEntry minOGainEntry = Constants.grabberDebugTab.add("Min Output", 0).getEntry();
  GenericEntry setPoint = Constants.grabberDebugTab.add("Set Point", 0).getEntry();
  GenericEntry rotationEntry = Constants.grabberDebugTab.add("Rotations", 0).getEntry();

  boolean manualControl = false;

  public Grabber() {
      grabberMotor.restoreFactoryDefaults();
      grabberMotor.setIdleMode(IdleMode.kBrake);
      grabberMotor.setSoftLimit(SoftLimitDirection.kForward, Constants.GRABBER_FORWARD_LIMIT);
      grabberMotor.setSoftLimit(SoftLimitDirection.kReverse, Constants.GRABBER_REVERSE_LIMIT);
      grabberMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
      grabberMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
      m_pidController = grabberMotor.getPIDController();

        // Encoder object created to display position values
    m_encoder = grabberMotor.getEncoder();

    // PID coefficients
    kP = 0.1; 
    kI = 0;
    kD = 0; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 0.5; 
    kMinOutput = -0.5;

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);
    

  }

  public void setManualControl(){
      manualControl = true;
  }

  public void setAutoControl(){
      manualControl = false;
  }

  public void setPosition(double pos){
      rotations = pos;
  }

  public void grabberManual(double voltage){
     grabberMotor.setVoltage(voltage);
  }

  public boolean atSetpoint(){
    if(Math.abs(rotations - this.grabberMotor.getEncoder().getPosition()) < 1.0){
        return true;
    }
    return false;
  }

  @Override
  public void periodic() {
    if(!manualControl){
      grabberMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
      grabberMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
      m_pidController.setReference(rotations, CANSparkMax.ControlType.kPosition);
    }
    else{
      grabberMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
      grabberMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
    }
    
     setPoint.setDouble(rotations);
     rotationEntry.setDouble(m_encoder.getPosition());

  }
}
