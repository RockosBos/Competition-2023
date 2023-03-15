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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//This subsystem will control the intake arm that work to bring a game piece into the robot. This includes the extension and the intake rollers.

/*
 * Components *
 *  intakeRoller Motor (15) - Spark Max Canbus powering a NEO 550.
 *  intakeExtend Motor (16) - Spark Max Canbus powering a NEO.
 *  intakeExtendLimit (2) - Digital Input limit switch.
 *  intakeRetractLimit (3) - Digital Input limit switch.
 * 
 * Description *
 *  The intake will extend out until the intake extend ls is pressed. Then the rollers will begin spinning.
 *  It will retract once the intake button is no longer being pressed. It will be required 
 * 
 * Commands *
 *  Stop Intake (default) - Intake is retracted and roller wheels are halted.
 *  Activate Intake - Intake is moved into the extended position and the roller wheels are turned on.
 *  Manual Extention - Manually extends the intake.
 *  Manual Retraction - Manually retracts the intake.
 * 
 * Functions *
 *  SetIntakeRollers - turns on intake rollers
 *  SetIntakeExtension - Runs intake extension as long as limits are not reached.
 *  isIntakeRetracted - returns true/false on whether the intake is extended or not.
 */

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private CANSparkMax intakeExtend = new CANSparkMax(Constants.intakeExtendID, MotorType.kBrushless);
  private CANSparkMax intakeRollerTop = new CANSparkMax(Constants.intakeRollerTopID, MotorType.kBrushless);
  private CANSparkMax IntakeRollerBottom = new CANSparkMax(Constants.intakeRollerBottomID,MotorType.kBrushless);
  private GenericEntry extensionPositionEntry, intakeZeroLimitEntry;

  private SparkMaxPIDController m_pidController = intakeExtend.getPIDController();
  private RelativeEncoder m_encoder = intakeExtend.getEncoder();
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, pos;
  private Timer intakeDelayTimer = new Timer();
  

  public Intake() {
    intakeExtend.clearFaults();
    intakeExtend.restoreFactoryDefaults();
    intakeExtend.setOpenLoopRampRate(Constants.INTAKE_EXTEND_RAMP_RATE);
    intakeExtend.enableSoftLimit(SoftLimitDirection.kForward, true);
    intakeExtend.enableSoftLimit(SoftLimitDirection.kReverse, true);
    intakeExtend.setSoftLimit(SoftLimitDirection.kForward, Constants.INTAKE_FORWARD_LIMIT);
    intakeExtend.setSoftLimit(SoftLimitDirection.kReverse, Constants.INTAKE_REVERSE_LIMIT);
    intakeExtend.setInverted(false);
    intakeExtend.setIdleMode(IdleMode.kBrake);

    intakeRollerTop.clearFaults();
    intakeRollerTop.restoreFactoryDefaults();
    intakeRollerTop.setInverted(false);
    //intakeRollerTop.setOpenLoopRampRate(Constants.INTAKE_ROLLER_RAMP_RATE);

    IntakeRollerBottom.clearFaults();
    IntakeRollerBottom.restoreFactoryDefaults();
    IntakeRollerBottom.setInverted(false);
    //IntakeRollerBottom.setOpenLoopRampRate(Constants.INTAKE_ROLLER_RAMP_RATE);
    //IntakeRollerBottom.follow(intakeRollerTop);

    intakeDelayTimer.start();

    // PID coefficients
    kP = 1; 
    kI = 0;
    kD = 0; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 0.4; 
    kMinOutput = -0.4;

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);
    

    extensionPositionEntry = Constants.intakeDebugTab.add("Extension Rotate Position", 0).getEntry();
    //intakeZeroLimitEntry = Constants.intakeDebugTab.add("Intake Zero Limit", intakeZeroLimit.get()).getEntry();

    pos = 0.0;
  } 

  public void SetIntakeRollers(double voltage) {
    if(Math.abs(intakeExtend.getEncoder().getPosition() - Constants.INTAKE_EXTEND_POSITION) < 3.0){
        intakeRollerTop.setVoltage(voltage);
        IntakeRollerBottom.setVoltage(voltage);
    }
    else{
      intakeRollerTop.setVoltage(0.0);
      IntakeRollerBottom.setVoltage(0.0);
    }
  }

  public void SetIntakePosition(double pos){
      this.pos = pos;
  }

  public void SetIntakeExtensionManual(double voltage){
    intakeExtend.setVoltage(voltage);
  }

  public boolean atSetpoint(){
    if(Math.abs(intakeExtend.getEncoder().getPosition() - pos) < 3.0){
      return true;
    }
    else{
      return false;
    }
  }

  @Override
  public void periodic() {

    //if(intakeZeroLimit.get()){
      //intakeExtend.getEncoder().setPosition(0.0);
    //}

    
    m_pidController.setReference(pos, CANSparkMax.ControlType.kPosition);

    extensionPositionEntry.setDouble(intakeExtend.getEncoder().getPosition());
    //intakeZeroLimitEntry.setBoolean(intakeZeroLimit.get());

  }
}
