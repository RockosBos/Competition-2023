// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class ManualExtendIntake extends CommandBase {

  private Intake s_Intake;
  private boolean liftExtended;

  public ManualExtendIntake(Intake s_Intake, boolean liftExtended) {
    this.s_Intake = s_Intake;
    this.liftExtended = liftExtended;
    addRequirements(this.s_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!liftExtended){
        this.s_Intake.SetIntakeExtensionManual(Constants.INTAKE_EXTENTION_SPEED_VOLTS);
        
    }
    this.s_Intake.SetIntakeRollers(Constants.INTAKE_ROLLER_SPEED_VOLTS);
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
