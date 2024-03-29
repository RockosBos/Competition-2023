// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;

public class LimeLightSearchOn extends CommandBase {
  /** Creates a new LimeLightSearchOn. */
  private Limelight s_Limelight;
  public LimeLightSearchOn(Limelight s_Limelight) {
    this.s_Limelight = s_Limelight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Limelight.setLEDOnState(true);
    s_Limelight.setLowExposure(true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
