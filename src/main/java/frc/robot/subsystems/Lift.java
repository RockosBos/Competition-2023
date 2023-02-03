// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

  public Lift() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
