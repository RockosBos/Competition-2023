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
    private GenericEntry rotate_PEntry, rotate_IEntry, rotate_DEntry, rotate_IZEntry, rotate_FFEntry, rotate_MaxOEntry, rotate_MinOEntry, rotate_RotEntry;
    private GenericEntry extend_PEntry, extend_IEntry, extend_DEntry, extend_IZEntry, extend_FFEntry, extend_MaxOEntry, extend_MinOEntry, extend_RotEntry;
    private GenericEntry rotatePositionEntry, extendPositionEntry, rotateSetPointEntry, extendSetPointEntry;
    private double rotateSetpoint, extendSetpoint;
    private DigitalInput zeroLimit;

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
      extend_kMaxOutput = 0.5; 
      extend_kMinOutput = -0.5;

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

      rotate_PEntry = Constants.pidConfigTab.add("Lift Rotate P Gain", rotate_kP).getEntry();
      rotate_IEntry = Constants.pidConfigTab.add("Lift Rotate I Gain", rotate_kI).getEntry();
      rotate_DEntry = Constants.pidConfigTab.add("Lift Rotate D Gain", rotate_kD).getEntry();
      rotate_IZEntry = Constants.pidConfigTab.add("Lift Rotate I Zone", rotate_kIz).getEntry();
      rotate_FFEntry = Constants.pidConfigTab.add("Lift Rotate FF", rotate_kFF).getEntry();
      rotate_MaxOEntry = Constants.pidConfigTab.add("Lift Rotate Max Out", rotate_kMaxOutput).getEntry();
      rotate_MinOEntry = Constants.pidConfigTab.add("Lift Rotate Min Out", rotate_kMinOutput).getEntry();
      rotate_RotEntry = Constants.pidConfigTab.add("Lift Rotate Set Rotations", 0).getEntry();

      extend_PEntry = Constants.pidConfigTab.add("Lift Extend P Gain", extend_kP).getEntry();
      extend_IEntry = Constants.pidConfigTab.add("Lift Extend I Gain", extend_kI).getEntry();
      extend_DEntry = Constants.pidConfigTab.add("Lift Extend D Gain", extend_kD).getEntry();
      extend_IZEntry = Constants.pidConfigTab.add("Lift Extend I Zone", extend_kIz).getEntry();
      extend_FFEntry = Constants.pidConfigTab.add("Lift Extend FF", extend_kFF).getEntry();
      extend_MaxOEntry = Constants.pidConfigTab.add("Lift Extend Max Out", extend_kMaxOutput).getEntry();
      extend_MinOEntry = Constants.pidConfigTab.add("Lift Extend Min Out", extend_kMinOutput).getEntry();
      extend_RotEntry = Constants.pidConfigTab.add("Lift Extend Set Rotations", 0).getEntry();

      rotatePositionEntry = Constants.liftDebugTab.add("Lift Rotate Current Position", 0).getEntry();
      extendPositionEntry = Constants.liftDebugTab.add("Extend Rotate Current Position", 0).getEntry();
      rotateSetPointEntry = Constants.liftDebugTab.add("Lift Rotate Set Point", 0).getEntry();
      extendSetPointEntry = Constants.liftDebugTab.add("Lift Extend Set Point", 0).getEntry();

      rotateSetpoint = 0;
      extendSetpoint = 0;
    }

    public void setPosition(double rotateSetpoint, double extendSetpoint){
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
      if(Math.abs(rotateSetpoint - this.getLiftRotatePosition()) < 1.0 /*&& Math.abs(extendSetpoint - this.getLiftRotatePosition()) < 1.0*/){
          return true;
      }
      return false;
    }

    @Override
    public void periodic() {

        double rotate_P = rotate_PEntry.getDouble(0);
        double rotate_I = rotate_IEntry.getDouble(0);
        double rotate_D = rotate_DEntry.getDouble(0);
        double rotate_IZ = rotate_IZEntry.getDouble(0);
        double rotate_FF = rotate_FFEntry.getDouble(0);
        double rotate_MaxO = rotate_MaxOEntry.getDouble(0);
        double rotate_MinO = rotate_MinOEntry.getDouble(0);

        double extend_P = extend_PEntry.getDouble(0);
        double extend_I = extend_IEntry.getDouble(0);
        double extend_D = extend_DEntry.getDouble(0);
        double extend_IZ = extend_IZEntry.getDouble(0);
        double extend_FF = extend_FFEntry.getDouble(0);
        double extend_MaxO = extend_MaxOEntry.getDouble(0);
        double extend_MinO = extend_MinOEntry.getDouble(0);
      /*
        if((rotate_P != rotate_kP)) { liftRotateController.setP(rotate_P); rotate_kP = rotate_P; }
        if((rotate_I != rotate_kI)) { liftRotateController.setI(rotate_I); rotate_kI = rotate_I; }
        if((rotate_D != rotate_kD)) { liftRotateController.setD(rotate_D); rotate_kD = rotate_D; }
        if((rotate_IZ != rotate_kIz)) { liftRotateController.setIZone(rotate_IZ); rotate_kIz = rotate_IZ; }
        if((rotate_FF != rotate_kFF)) { liftRotateController.setFF(rotate_FF); rotate_kFF = rotate_FF; }
        if((rotate_MaxO != rotate_kMaxOutput) || (rotate_MinO != rotate_kMinOutput)) { 
          liftRotateController.setOutputRange(rotate_MinO, rotate_MaxO); 
          rotate_kMinOutput = rotate_MinO; rotate_kMaxOutput = rotate_MaxO; 
        }

        if((extend_P != extend_kP)) { liftExtendController.setP(extend_P); extend_kP = extend_P; }
        if((extend_I != extend_kI)) { liftExtendController.setI(extend_I); extend_kI = extend_I; }
        if((extend_D != extend_kD)) { liftExtendController.setD(extend_D); extend_kD = extend_D; }
        if((extend_IZ != extend_kIz)) { liftExtendController.setIZone(extend_IZ); extend_kIz = extend_IZ; }
        if((extend_FF != extend_kFF)) { liftExtendController.setFF(extend_FF); extend_kFF = extend_FF; }
        if((extend_MaxO != extend_kMaxOutput) || (extend_MinO != extend_kMinOutput)) { 
          liftExtendController.setOutputRange(extend_MinO, extend_MaxO); 
          extend_kMinOutput = extend_MinO; extend_kMaxOutput = extend_MaxO; 
        }

        /*
        if(zeroLimit.get()){
          liftRotateEncoder.setPosition(0.0);
          liftExtendEncoder.setPosition(0.0);
        }
        */

        if(liftRotate.getEncoder().getPosition() < Constants.LIFT_ROTATE_CLEAR_POSITION){
          extendSetpoint = Constants.LIFT_EXTEND_CLEAR_POSITION;
        }
        /*
        if(liftExtend.getEncoder().getPosition() > Constants.LIFT_EXTEND_CLEAR_POSITION){
          rotateSetpoint = Constants.LIFT_ROTATE_CLEAR_POSITION;
        }
        */

        liftRotateController.setReference(rotateSetpoint, CANSparkMax.ControlType.kPosition);
        liftExtendController.setReference(extendSetpoint, CANSparkMax.ControlType.kPosition);

        rotatePositionEntry.setDouble(getLiftRotatePosition());
        extendPositionEntry.setDouble(getLiftExtendPosition());
        rotateSetPointEntry.setDouble(rotateSetpoint);
        extendSetPointEntry.setDouble(extendSetpoint);

        
    }
}
