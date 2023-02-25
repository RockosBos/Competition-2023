// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Score2Adjacent extends SequentialCommandGroup {
  /** Creates a new Score2Left. */
  Swerve s_Swerve;
  public Score2Adjacent(Swerve s_Swerve) {
    this.s_Swerve = s_Swerve;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      s_Swerve.followTrajectoryCommand(PathPlanner.loadPath("Score2Adjacent", 1, 1), true)
    );
  }
}
