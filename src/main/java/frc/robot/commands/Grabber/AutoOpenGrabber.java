// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Grabber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Grabber;

public class AutoOpenGrabber extends CommandBase {
  Grabber s_Grabber;
  public AutoOpenGrabber(Grabber s_Grabber) {
    this.s_Grabber = s_Grabber;
    addRequirements(s_Grabber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      this.s_Grabber.setPosition(Constants.GRABBER_OPEN_POSITION);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.s_Grabber.atSetpoint();
  }
}
