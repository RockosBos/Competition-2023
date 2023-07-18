// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Conveyor.SetServoLow;
import frc.robot.commands.Conveyor.TurnOffConveyor;
import frc.robot.commands.Conveyor.TurnOnConveyor;
import frc.robot.commands.Grabber.AutoCloseGrabber;
import frc.robot.commands.Grabber.AutoOpenGrabber;
import frc.robot.commands.Grabber.CloseGrabber;
import frc.robot.commands.Grabber.OpenGrabber;
import frc.robot.commands.Intake.ExtendIntake;
import frc.robot.commands.Intake.RetractIntake;
import frc.robot.commands.Lift.AutoScoreLevel0;
import frc.robot.commands.Lift.AutoScoreLevel3;
import frc.robot.commands.Lift.SetPosition0;
import frc.robot.commands.Lift.SetPositionDrop;
import frc.robot.commands.Lift.SetPositionGrab;
import frc.robot.commands.Limelight.LimeLightSearchOff;
import frc.robot.commands.Limelight.LimeLightSearchOn;
import frc.robot.commands.Swerve.AutoCenter;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Score1_5Adjacent extends SequentialCommandGroup {
  /** Creates a new Score2Right. */
  Swerve s_Swerve;
  Lift s_Lift;
  Intake s_Intake;
  Grabber s_Grabber;
  Limelight s_Limelight;
  Conveyor s_Conveyor;

  public Score1_5Adjacent(Swerve s_Swerve, Lift s_Lift, Intake s_Intake, Grabber s_Grabber, Limelight s_Limelight, Conveyor s_Conveyor) {
    this.s_Swerve = s_Swerve;
    this.s_Lift = s_Lift;
    this.s_Intake = s_Intake;
    this.s_Grabber = s_Grabber;
    this.s_Limelight = s_Limelight;
    this.s_Conveyor = s_Conveyor;
    addCommands(
        new SetZeroPoints(s_Lift),
        new SetServoLow(s_Conveyor),
        new SetPositionGrab(s_Lift, s_Grabber),
        new AutoCloseGrabber(s_Grabber),
        new AutoScoreLevel3(s_Lift),
        new ParallelCommandGroup(
          new LimeLightSearchOn(s_Limelight), 
          new AutoCenter(s_Swerve, s_Limelight)
        ), 
        new DelayCommand(0.5),
        new SetPositionDrop(s_Lift),
        new OpenGrabber(s_Grabber), 
        new ParallelCommandGroup(
            new SetPosition0(s_Lift),
            new LimeLightSearchOff(s_Limelight),
            s_Swerve.followTrajectoryCommand(PathPlanner.loadPath("Score1_5Adjacent", 5, 2.5), true), 
            new ExtendIntake(s_Intake),
            new TurnOnConveyor(s_Conveyor)
        )
    );
  }
}



