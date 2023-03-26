// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class AutoCenter extends CommandBase {

  private Swerve s_Swerve;
  private Limelight s_Limelight;
  private double translationX;
  private double driveSpeed, rotationSpeed;

  public AutoCenter(Swerve s_Swerve, Limelight s_Limelight) {
      this.s_Swerve = s_Swerve;
      this.s_Limelight = s_Limelight;
      driveSpeed = 0;
      rotationSpeed = 0;
      addRequirements(s_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      translationX = s_Limelight.getSampleAverage() - Constants.LIMELIGHT_TX_OFFSET;

      if(translationX > Constants.LIMELIGHT_STRAFE_ERROR_MARGIN){
          driveSpeed = -0.10;
      }
      else if(translationX < -Constants.LIMELIGHT_STRAFE_ERROR_MARGIN){
          driveSpeed = 0.10;
      }
      else{
          driveSpeed = -0.0;
      }

      /*if(s_Swerve.gyro.getYaw() > 1){
          rotationSpeed = -0.1;
          driveSpeed = 0.0;
      }
      else if(s_Swerve.gyro.getYaw() < -1){
          rotationSpeed = 0.1;
          driveSpeed = 0.0;
      }
      else{
          rotationSpeed = 0.0;
      }*/

      this.s_Swerve.drive(
        new Translation2d(0.0, -driveSpeed).times(Constants.Swerve.maxSpeed), 
        rotationSpeed * Constants.Swerve.maxAngularVelocity, 
        false,
        true
      );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println(Math.abs(s_Limelight.getSampleAverage() - Constants.LIMELIGHT_TX_OFFSET));
    return (Math.abs(s_Limelight.getSampleAverage() - Constants.LIMELIGHT_TX_OFFSET) < Constants.LIMELIGHT_STRAFE_ERROR_MARGIN);
  }
}
