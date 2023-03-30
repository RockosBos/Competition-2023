// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Swerve;

public class DoubleSubstationDriveUp extends CommandBase {
  /** Creates a new DoubleSubstationDriveUp. */
  Swerve s_Swerve;
  Lift s_Lift;
  double translationValue;
  Timer goodStatetimer;

  public DoubleSubstationDriveUp(Swerve s_Swerve, Lift s_Lift) {
    this.s_Swerve = s_Swerve;
    this.s_Lift = s_Lift;
    addRequirements(this.s_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      translationValue = 0.0;
      goodStatetimer = new Timer();
      goodStatetimer.start();
      
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if(this.s_Lift.getSonicSensorDistance() - Constants.SONIC_TARGET_DISTANCE > Constants.SONIC_TARGETING_ERROR_MARGIN){
        translationValue = Constants.SONIC_TARGETING_DRIVE_SPEED;
        goodStatetimer.reset();
      }
      else if(this.s_Lift.getSonicSensorDistance() - Constants.SONIC_TARGET_DISTANCE < -Constants.SONIC_TARGETING_ERROR_MARGIN) {
        translationValue = -Constants.SONIC_TARGETING_DRIVE_SPEED;
        goodStatetimer.reset();
      }
      else {
        translationValue = 0.0;
      }

      s_Swerve.drive(
        new Translation2d(translationValue, 0.0),
        0.0,
        true,
        true
      );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      goodStatetimer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(goodStatetimer.get() > 0.25){
        return true;
    }
    return false;
  }
}
