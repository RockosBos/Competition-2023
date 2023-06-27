// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Lift;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lift;

public class DisableManualControl extends CommandBase {
  /** Creates a new DisableManualControl. */
  Lift s_Lift;
  boolean complete = false;
  public DisableManualControl(Lift s_Lift) {
    this.s_Lift = s_Lift;
    addRequirements(s_Lift);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Lift.setPosition(s_Lift.getLiftRotatePosition(), s_Lift.getLiftExtendPosition());
    s_Lift.setPositionControl(true);
    complete = true;
    System.out.println("Manual Control off");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return complete;
  }
}
