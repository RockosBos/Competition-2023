// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
 *  
 */

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  public Intake() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
