// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Lift;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Lift;

public class SetPositionDrop extends CommandBase {
  /** Creates a new SetPositionDrop. */
  private Lift s_Lift;
  private double currentSetpoint, newSetpoint;
  public SetPositionDrop(Lift s_Lift) {
    // Use addRequirements() here to declare subsystem dependencies.
      this.s_Lift = s_Lift;
      addRequirements(this.s_Lift);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      currentSetpoint = s_Lift.getLiftRotateSetpoint();
      newSetpoint = currentSetpoint - Constants.LIFT_SETPOINT_DROP;
      s_Lift.setDropState(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      s_Lift.setPosition(newSetpoint, s_Lift.getLiftExtendSetpoint());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Lift.setDropState(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(s_Lift.atSetpoint()){
      return true;
    }
    return false;
  }
}
