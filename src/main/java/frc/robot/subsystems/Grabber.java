// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Grabber extends SubsystemBase {
  /** Creates a new Grabber. */

  private CANSparkMax grabberMotor = new CANSparkMax(Constants.grabberID, MotorType.kBrushless);

  public Grabber() {
      grabberMotor.restoreFactoryDefaults();
      grabberMotor.setIdleMode(IdleMode.kBrake);
      grabberMotor.setSoftLimit(SoftLimitDirection.kForward, Constants.GRABBER_FORWARD_LIMIT);
      grabberMotor.setSoftLimit(SoftLimitDirection.kReverse, Constants.GRABBER_REVERSE_LIMIT);

  }

  public void setPosition(int pos){
    
  }

  public void grabberManual(double speed){
     grabberMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
