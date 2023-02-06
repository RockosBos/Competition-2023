// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class ExtendIntake extends CommandBase {
  /** Creates a new ExtendIntake. */
  private Intake s_Intake;

  public ExtendIntake(Intake s_Intake) {
    this.s_Intake = s_Intake;
    addRequirements(this.s_Intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.s_Intake.SetIntakeExtension(Constants.INTAKE_EXTENTION_SPEED);
    this.s_Intake.SetIntakeRollers(Constants.INTAKE_ROLLER_SPEED);
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

