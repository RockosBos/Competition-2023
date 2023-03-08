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
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Intake.ExtendIntake;

public class Lift extends SubsystemBase {
  /** Creates a new Lift. */

  //This subsystem will control the lift that will carry game pieces to scoring zones.

/*
 * Components *
 *  liftRotate Motor (17) - Can SparkMax powering a NEO.
 *  liftExtend Motor (18) - Can sparkMax powering a NEO.
 *  liftExtendRetract Limit (4) - Digital Input Limit Switch.
 *  liftRotateRetract Limit (5) - Digital Input Limit Switch.
 * 
 * Description *
 *  The lift will move to a series of set positions in order to score pieces at different levels. This will
 *  be controlled by the built in NEO encoders and the position will be reset when the lower limit switch
 *  is pressed.
 * 
 * Commands *
 *  setRetrievalPosition - Arm is moved to low position where it can grab a new game piece.
 *  setScoringLevel1 - arm is moved to position to score at level 1.
 *  setScoringLevel2 - arm is moved to position to score at level 2.
 *  setScoringLevel3 - arm is moved to position to score at level 3.
 * 
 * Functions *
 *  setPosition
 */

    private CANSparkMax liftRotate = new CANSparkMax(Constants.liftRotateID, MotorType.kBrushless);
    private CANSparkMax liftExtend = new CANSparkMax(Constants.liftExtendID, MotorType.kBrushless);
    private SparkMaxPIDController liftRotateController;
    private SparkMaxPIDController liftExtendController;
    private double rotate_kP, rotate_kI, rotate_kD, rotate_kIz, rotate_kFF, rotate_kMaxOutput, rotate_kMinOutput;
    private double extend_kP, extend_kI, extend_kD, extend_kIz, extend_kFF, extend_kMaxOutput, extend_kMinOutput;
    private GenericEntry rotatePositionEntry, extendPositionEntry, rotateSetPointEntry, extendSetPointEntry;
    private double rotateSetpoint, extendSetpoint;
    private DigitalInput zeroLimit;
    private boolean manualControl;
    private int setpointLevel;
    private double lastExtendSetpoint;

    public Lift() {
      liftRotate.restoreFactoryDefaults();
      liftExtend.restoreFactoryDefaults();
      liftRotate.setIdleMode(IdleMode.kBrake);
      liftExtend.setIdleMode(IdleMode.kBrake);
      liftRotate.enableSoftLimit(SoftLimitDirection.kForward, true);
      liftRotate.enableSoftLimit(SoftLimitDirection.kReverse, true);
      liftExtend.enableSoftLimit(SoftLimitDirection.kForward, true);
      liftExtend.enableSoftLimit(SoftLimitDirection.kReverse, true);
      liftExtend.setSoftLimit(SoftLimitDirection.kForward, Constants.LIFT_EXTEND_FORWARD_LIMIT);
      liftExtend.setSoftLimit(SoftLimitDirection.kReverse, Constants.LIFT_EXTEND_REVERSE_LIMIT);
      liftRotate.setSoftLimit(SoftLimitDirection.kForward, Constants.LIFT_ROTATE_FORWARD_LIMIT);
      liftRotate.setSoftLimit(SoftLimitDirection.kReverse, Constants.LIFT_ROTATE_REVERSE_LIMIT);
      liftExtend.setInverted(true);

      liftRotateController = liftRotate.getPIDController();
      liftExtendController = liftExtend.getPIDController();

      manualControl = false;
      setpointLevel = 0;

      //PID Setup

      rotate_kP = 1; 
      rotate_kI = 0;
      rotate_kD = 0; 
      rotate_kIz = 0; 
      rotate_kFF = 0; 
      rotate_kMaxOutput = .75; 
      rotate_kMinOutput = -.75;

      extend_kP = 1; 
      extend_kI = 0;
      extend_kD = 0; 
      extend_kIz = 0; 
      extend_kFF = 0; 
      extend_kMaxOutput = 0.75; 
      extend_kMinOutput = -0.75;

      liftRotateController.setP(rotate_kP);
      liftRotateController.setI(rotate_kI);
      liftRotateController.setD(rotate_kD);
      liftRotateController.setIZone(rotate_kIz);
      liftRotateController.setFF(rotate_kFF);
      liftRotateController.setOutputRange(rotate_kMinOutput, rotate_kMaxOutput);

      liftExtendController.setP(extend_kP);
      liftExtendController.setI(extend_kI);
      liftExtendController.setD(extend_kD);
      liftExtendController.setIZone(extend_kIz);
      liftExtendController.setFF(extend_kFF);
      liftExtendController.setOutputRange(extend_kMinOutput, extend_kMaxOutput);

      rotatePositionEntry = Constants.liftDebugTab.add("Lift Rotate Current Position", 0).getEntry();
      extendPositionEntry = Constants.liftDebugTab.add("Extend Rotate Current Position", 0).getEntry();
      rotateSetPointEntry = Constants.liftDebugTab.add("Lift Rotate Set Point", 0).getEntry();
      extendSetPointEntry = Constants.liftDebugTab.add("Lift Extend Set Point", 0).getEntry();

      rotateSetpoint = 0;
      extendSetpoint = 0;
    }

    public void setPosition(double rotateSetpoint, double extendSetpoint){

        lastExtendSetpoint = this.extendSetpoint;
        this.rotateSetpoint = rotateSetpoint;
        this.extendSetpoint = extendSetpoint;

    }

    public boolean isLiftRetracted(){
      return true;
    }

    public double getLiftRotatePosition(){
      return this.liftRotate.getEncoder().getPosition();
    }

    public double getLiftExtendPosition(){
      return this.liftExtend.getEncoder().getPosition();
    }

    public boolean atSetpoint(){
      if(Math.abs(rotateSetpoint - this.getLiftRotatePosition()) < 1.0 && Math.abs(extendSetpoint - this.getLiftRotatePosition()) < 1.0){
          return true;
      }
      return false;
    }

    public boolean atRotateSetpoint(){
      if(Math.abs(rotateSetpoint - this.getLiftRotatePosition()) < 1.0){
          return true;
      }
      return false;
    }

    public boolean atExtendSetpoint(){
      if(Math.abs(extendSetpoint - this.getLiftExtendPosition()) < 1.0){
          return true;
      }
      return false;
    }

    public void setManualControl(){
        manualControl = true;
    }

    public void setAutoControl(){
        manualControl = false;
    }

    public void setManualExtendVoltage(double voltage){
        liftExtend.setVoltage(voltage);
    }
    public void setManualRotateVoltage(double voltage){
        liftRotate.setVoltage(voltage);
  }

    @Override
    public void periodic() {

        /*
        if(zeroLimit.get()){
          liftRotateEncoder.setPosition(0.0);
          liftExtendEncoder.setPosition(0.0);
        }
        */
        if(!manualControl){
            liftExtend.enableSoftLimit(SoftLimitDirection.kForward, true);
            liftExtend.enableSoftLimit(SoftLimitDirection.kReverse, true);
            liftRotate.enableSoftLimit(SoftLimitDirection.kForward, true);
            liftRotate.enableSoftLimit(SoftLimitDirection.kReverse, true);

            if(atExtendSetpoint()){
              liftRotateController.setReference(rotateSetpoint, CANSparkMax.ControlType.kPosition);
            }
            else{
              liftRotateController.setReference(getLiftRotatePosition(), CANSparkMax.ControlType.kPosition);
            }
            if(atRotateSetpoint()){
              liftExtendController.setReference(extendSetpoint, CANSparkMax.ControlType.kPosition);
            }
            else{
              liftExtendController.setReference(Constants.LIFT_EXTEND_POSITION_0, CANSparkMax.ControlType.kPosition);
            }
            
        }
        else{
            liftExtend.enableSoftLimit(SoftLimitDirection.kForward, false);
            liftExtend.enableSoftLimit(SoftLimitDirection.kReverse, false);
            liftRotate.enableSoftLimit(SoftLimitDirection.kForward, false);
            liftRotate.enableSoftLimit(SoftLimitDirection.kReverse, false);
        }
        
        

        rotatePositionEntry.setDouble(getLiftRotatePosition());
        extendPositionEntry.setDouble(getLiftExtendPosition());
        rotateSetPointEntry.setDouble(rotateSetpoint);
        extendSetPointEntry.setDouble(extendSetpoint);

        
    }
}
