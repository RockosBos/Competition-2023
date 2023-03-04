// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class RetractIntake extends CommandBase {
  /** Creates a new RetractIntake. */

  private Intake s_Intake;

  private Timer timer = new Timer();
  
  public RetractIntake(Intake s_Intake) {
    this.s_Intake = s_Intake;
    addRequirements(this.s_Intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(timer.get() < 2.0){
        this.s_Intake.SetIntakeExtension(0.0);
    }
    else{
        this.s_Intake.SetIntakeExtension(Constants.INTAKE_RETRACTION_SPEED_VOLTS);
    }
    
    this.s_Intake.SetIntakeRollers(0.0);
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








