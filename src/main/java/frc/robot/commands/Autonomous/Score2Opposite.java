// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Grabber.AutoCloseGrabber;
import frc.robot.commands.Grabber.AutoOpenGrabber;
import frc.robot.commands.Intake.ExtendIntake;
import frc.robot.commands.Lift.AutoScoreLevel0;
import frc.robot.commands.Lift.AutoScoreLevel3;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Score2Opposite extends SequentialCommandGroup {
  /** Creates a new Score2Right. */
  Swerve s_Swerve;
  Lift s_Lift;
  Intake s_Intake;
  Grabber s_Grabber;

  public Score2Opposite(Swerve s_Swerve, Lift s_Lift, Intake s_Intake, Grabber s_Grabber) {
    this.s_Swerve = s_Swerve;
    this.s_Lift = s_Lift;
    this.s_Intake = s_Intake;
    this.s_Grabber = s_Grabber;
    addCommands(
      new AutoCloseGrabber(s_Grabber),
      new AutoScoreLevel3(s_Lift),
      new AutoOpenGrabber(s_Grabber),
      new AutoScoreLevel0(s_Lift),
      new ParallelCommandGroup(
            s_Swerve.followTrajectoryCommand(PathPlanner.loadPath("Score2Opposite", 1, 1), true), 
            s_Intake.run(() -> new ExtendIntake(s_Intake, !s_Lift.isLiftRetracted()))
        ),
        new AutoCloseGrabber(s_Grabber),
        new AutoScoreLevel3(s_Lift),
        new AutoOpenGrabber(s_Grabber)
    );
  }
}
