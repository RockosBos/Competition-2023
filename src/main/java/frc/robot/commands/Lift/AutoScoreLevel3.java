// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Lift;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Lift;

public class AutoScoreLevel3 extends CommandBase {
  
  private Lift s_Lift;
  /** Creates a new AutoScoreLevel3. */
  public AutoScoreLevel3(Lift s_Lift) {
      this.s_Lift = s_Lift;
      addRequirements(this.s_Lift);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      s_Lift.setPosition(Constants.LIFT_ROTATE_POSITION_3, Constants.LIFT_EXTEND_POSITION_3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return s_Lift.atSetpoint();
  }
}
