// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Grabber.AutoCloseGrabber;
import frc.robot.commands.Grabber.AutoOpenGrabber;
import frc.robot.commands.Lift.AutoScoreLevel0;
import frc.robot.commands.Lift.AutoScoreLevel3;
import frc.robot.commands.Swerve.AutoBalance;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreBalanceAdjacent extends SequentialCommandGroup {
  /** Creates a new ScoreBalanceLeft. */
  Swerve s_Swerve;
  Lift s_Lift;
  Grabber s_Grabber;
  public ScoreBalanceAdjacent(Swerve s_Swerve, Lift s_Lift, Grabber s_Grabber) {
    this.s_Swerve = s_Swerve;
    this.s_Lift = s_Lift;
    this.s_Grabber = s_Grabber;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

      new ParallelCommandGroup(
          new AutoScoreLevel3(s_Lift),
          new AutoCloseGrabber(s_Grabber)
        ),
        new ParallelCommandGroup(
          new AutoScoreLevel0(s_Lift),
          new AutoOpenGrabber(s_Grabber)  
        ),
      s_Swerve.followTrajectoryCommand(PathPlanner.loadPath("ScoreBalanceAdjacent", 1, 1), true),
      new AutoBalance(s_Swerve)
    );
  }
}
