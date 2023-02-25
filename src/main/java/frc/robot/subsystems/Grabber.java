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
    kI = 1e-4;
    kD = 1; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);
    

  }

  public void setPosition(double pos){
      rotations = pos;
  }

  public void grabberManual(double speed){
     grabberMotor.set(speed);
  }

  @Override
  public void periodic() {
    double p = pGainEntry.getDouble(0);
    double i = iGainEntry.getDouble(0);
    double d = dGainEntry.getDouble(0);
    double iz = iZGainEntry.getDouble(0);
    //double ff = pGainEntry.getDouble(0);
    double max = maxOGainEntry.getDouble(0);
    double min = minOGainEntry.getDouble(0);
    //double rotations = SmartDashboard.getNumber("Set Rotations", 0);

     // if PID coefficients on SmartDashboard have changed, write new values to controller
     if((p != kP)) { m_pidController.setP(p); kP = p; }
     if((i != kI)) { m_pidController.setI(i); kI = i; }
     if((d != kD)) { m_pidController.setD(d); kD = d; }
     if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
     //if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
     if((max != kMaxOutput) || (min != kMinOutput)) { 
       m_pidController.setOutputRange(min, max); 
       kMinOutput = min; kMaxOutput = max; 
     }
    
     m_pidController.setReference(rotations, CANSparkMax.ControlType.kPosition);
    
     setPoint.setDouble(rotations);
     rotationEntry.setDouble(m_encoder.getPosition());

    // This method will be called once per scheduler run
  }
}
