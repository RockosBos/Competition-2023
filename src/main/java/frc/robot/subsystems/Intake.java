// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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
  private CANSparkMax intakeRoller = new CANSparkMax(Constants.intakeRollerID, MotorType.kBrushless);
  private DigitalInput intakeExtendedLimit = new DigitalInput(Constants.intakeExtendLimitID);
  private DigitalInput intakeRetractedLimit = new DigitalInput(Constants.intakeRetractLimitID);

  public Intake() {
    intakeExtend.clearFaults();
    intakeExtend.restoreFactoryDefaults();
    intakeExtend.setOpenLoopRampRate(Constants.INTAKE_EXTEND_RAMP_RATE);

    intakeRoller.clearFaults();
    intakeRoller.restoreFactoryDefaults();
    intakeRoller.setOpenLoopRampRate(Constants.INTAKE_ROLLER_RAMP_RATE);
  } 

  public void SetIntakeRollers(double voltage) {
    intakeRoller.setVoltage(voltage);
  }

  public void SetIntakeExtension(double voltage){
    if(voltage > 0 && intakeExtendedLimit.get()){
      intakeExtend.stopMotor();
    }
    else if(voltage < 0 && intakeRetractedLimit.get()){
      intakeExtend.stopMotor();
    } 
    else{
      intakeExtend.setVoltage(voltage);
    }
  }

  public void SetIntakeExtensionManual(double voltage){
    intakeExtend.setVoltage(voltage);
  }

  public boolean isIntakeRetracted(){
    return intakeRetractedLimit.get();
  }

  @Override
  public void periodic() {
    Constants.intakeDebugTab.add("Intake Extend CAN ID", intakeExtend.getDeviceId());
    Constants.intakeDebugTab.add("Intake Roller CAN ID", intakeRoller.getDeviceId());
    Constants.intakeDebugTab.add("Intake Extended Limit", intakeExtendedLimit.get());
    Constants.intakeDebugTab.add("Intake Retracted Limit", intakeRetractedLimit.get());

  }
}
