// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
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
  private DigitalInput intakeZeroLimit = new DigitalInput(Constants.intakeZeroLimitID);
  private GenericEntry extensionPositionEntry, intakeZeroLimitEntry;

  public Intake() {
    intakeExtend.clearFaults();
    intakeExtend.restoreFactoryDefaults();
    intakeExtend.setOpenLoopRampRate(Constants.INTAKE_EXTEND_RAMP_RATE);
    intakeExtend.enableSoftLimit(SoftLimitDirection.kForward, true);
    intakeExtend.enableSoftLimit(SoftLimitDirection.kReverse, true);
    intakeExtend.setSoftLimit(SoftLimitDirection.kForward, Constants.INTAKE_EXTEND_FORWARD_LIMIT);
    intakeExtend.setSoftLimit(SoftLimitDirection.kReverse, Constants.INTAKE_EXTEND_REVERSE_LIMIT);
    intakeExtend.setInverted(true);

    intakeRollerTop.clearFaults();
    intakeRollerTop.restoreFactoryDefaults();
    intakeRollerTop.setOpenLoopRampRate(Constants.INTAKE_ROLLER_RAMP_RATE);

    IntakeRollerBottom.clearFaults();
    IntakeRollerBottom.restoreFactoryDefaults();
    IntakeRollerBottom.setInverted(true);
    IntakeRollerBottom.setOpenLoopRampRate(Constants.INTAKE_ROLLER_RAMP_RATE);
    IntakeRollerBottom.follow(intakeRollerTop);

    extensionPositionEntry = Constants.intakeDebugTab.add("Extension Rotate Position", 0).getEntry();
    intakeZeroLimitEntry = Constants.intakeDebugTab.add("Intake Zero Limit", intakeZeroLimit.get()).getEntry();
  } 

  public void SetIntakeRollers(double voltage) {
    intakeRollerTop.setVoltage(voltage);
  }

  public void SetIntakeExtension(double voltage){
      //intakeExtend.setVoltage(voltage);
  }

  public void SetIntakeExtensionManual(double voltage){
    intakeExtend.setVoltage(voltage);
  }

  public boolean isIntakeRetracted(){
    return intakeZeroLimit.get();
  }

  @Override
  public void periodic() {

    if(intakeZeroLimit.get()){
      //intakeExtend.getEncoder().setPosition(0.0);
    }

    //extensionPositionEntry.setDouble(intakeExtend.getEncoder().getPosition());
    intakeZeroLimitEntry.setBoolean(intakeZeroLimit.get());

  }
}
