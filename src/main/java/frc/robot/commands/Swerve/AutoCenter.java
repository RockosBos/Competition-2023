// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class AutoCenter extends CommandBase {

  private Swerve s_Swerve;
  private double translationX;
  private double driveSpeed;

  public AutoCenter(Swerve s_Swerve, double translationX) {
      this.s_Swerve = s_Swerve;
      this.translationX = translationX;
      driveSpeed = 0;
      addRequirements(s_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if(Math.abs(translationX) < 0.5){
          driveSpeed = 0.0;
      }
      else if(translationX > 0.5){
          driveSpeed = 0.10;
      }
      else{
          driveSpeed = -0.10;
      }

      this.s_Swerve.drive(
        new Translation2d(driveSpeed, 0.0), 
        0.0, 
        true,
        true
      );
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
