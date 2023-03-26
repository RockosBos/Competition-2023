// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;

public class LimeLightSearchOff extends CommandBase {
  /** Creates a new LimeLightSearchOff. */
  private Limelight s_Limelight;
  boolean settingsSet;
  public LimeLightSearchOff(Limelight s_Limelight) {
    this.s_Limelight = s_Limelight;
    addRequirements(s_Limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    settingsSet = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      s_Limelight.setLEDOnState(false);
      s_Limelight.setLowExposure(false);
      settingsSet = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return settingsSet;
  }
}
