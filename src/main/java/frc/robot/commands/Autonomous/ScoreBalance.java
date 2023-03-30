// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Conveyor.SetServoLow;
import frc.robot.commands.Grabber.AutoCloseGrabber;
import frc.robot.commands.Grabber.AutoOpenGrabber;
import frc.robot.commands.Lift.AutoScoreLevel0;
import frc.robot.commands.Lift.AutoScoreLevel3;
import frc.robot.commands.Lift.SetPosition0;
import frc.robot.commands.Swerve.AutoBalance;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreBalance extends SequentialCommandGroup {
  /** Creates a new ScoreMobility. */
  Swerve s_Swerve;
  Lift s_Lift;
  Grabber s_Grabber;
  Conveyor s_Conveyor;
  public ScoreBalance(Swerve s_Swerve, Lift s_Lift, Grabber s_Grabber, Conveyor s_Conveyor) {
    this.s_Swerve = s_Swerve;
    this.s_Lift = s_Lift;
    this.s_Grabber = s_Grabber;
    this.s_Conveyor = s_Conveyor;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new SetZeroPoints(s_Lift),
        new AutoCloseGrabber(s_Grabber),
        new SetServoLow(s_Conveyor),
        new AutoScoreLevel3(s_Lift),
        new ParallelCommandGroup(
            new AutoOpenGrabber(s_Grabber), 
            new SetPosition0(s_Lift), 
            s_Swerve.followTrajectoryCommand(PathPlanner.loadPath("ScoreBalance", 1.25, 1.25), true)
        ),
        
        new AutoBalance(s_Swerve)
    );
  }
}
