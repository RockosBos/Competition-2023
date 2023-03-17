// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Grabber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Lift;

public class Position0GrabberControl extends CommandBase {
  /** Creates a new Position0GrabberControl. */
  Grabber s_Grabber;
  Lift s_Lift;
  public Position0GrabberControl(Grabber s_Grabber, Lift s_Lift) {
    this.s_Grabber = s_Grabber;
    this.s_Lift = s_Lift;
    addRequirements(s_Grabber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if((s_Lift.getLiftExtendSetpoint() == Constants.LIFT_EXTEND_POSITION_INTAKE && s_Lift.getLiftRotateSetpoint() == Constants.LIFT_ROTATE_POSITION_INTAKE) || (s_Lift.getLiftExtendSetpoint() == Constants.LIFT_EXTEND_POSITION_1 && s_Lift.getLiftRotateSetpoint() == Constants.LIFT_ROTATE_POSITION_1)){
        s_Grabber.setPosition(Constants.GRABBER_CLOSED_POSITION);
      }
      else{
        s_Grabber.setPosition(Constants.GRABBER_OPEN_POSITION);
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(s_Grabber.atSetpoint()){
      return true;
    }
    return false;
  }
}
