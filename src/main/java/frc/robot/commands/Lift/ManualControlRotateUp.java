// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Lift;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Lift;

public class ManualControlRotateUp extends CommandBase {
  /** Creates a new ManualControlRotateUp. */
  Lift s_Lift;
  public ManualControlRotateUp(Lift s_Lift) {
      this.s_Lift = s_Lift;
      addRequirements(s_Lift);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Lift.setPositionControl(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Lift.setRotateVoltage(Constants.LIFT_ROTATE_SPEED_VOLTS);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Lift.setPosition(s_Lift.getLiftRotatePosition(), s_Lift.getLiftExtendPosition());
    s_Lift.setPositionControl(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
