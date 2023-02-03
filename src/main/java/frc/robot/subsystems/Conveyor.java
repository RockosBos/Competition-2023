// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

//This class will control the conveyor belt that serves the purpose of positioning cubes and cones so that they can be grabbed by the lift.

/*
 * Components *
 *  Conveyor Motor(14) - Spark Max Canbus powering Neo 550.
 *  Intake Photoeye(0) - Digital Input sensor located on the front of the robot.
 *  Lift Photoeye(1) - Digital Input sensor located on the back of the robot.
 * 
 * Description *
 *  This subsystem by default will not be controlled by the driver or operator (outside manual control).
 *  Instead the intake sensor will detect if a game piece has entered the conveyor and the conveyor will move until
 *  the game piece has reached the lift side sensor.
 * 
 * Commands *
 *  setConveyor (Default) - Runs the functionality described above.
 *  setManualForward - Manually runs the conveyor forward.
 *  setManualBackward - Manually runs the conveyor backward.
 * 
 * Functions *
 *  setConveyor
 *  setConveyorManual
 *  get
 */

public class Conveyor extends SubsystemBase {
  /** Creates a new Conveyor. */

  public Conveyor() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
