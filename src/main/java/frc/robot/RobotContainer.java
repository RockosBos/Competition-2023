// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//This project utilizes a modified version of FIRST Robotics Competition Team 364's Open source Swerve Drive code. 
//This project can be accessed from the following link: https://github.com/Team364/BaseFalconSwerve

package frc.robot;

import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.Autonomous.Delay;
import frc.robot.commands.Autonomous.Mobility;
import frc.robot.commands.Autonomous.Score2Adjacent;
import frc.robot.commands.Autonomous.Score2Opposite;
import frc.robot.commands.Autonomous.ScoreBalance;
import frc.robot.commands.Autonomous.ScoreBalanceAdjacent;
import frc.robot.commands.Autonomous.ScoreBalanceOpposite;
import frc.robot.commands.Autonomous.SetZeroPoints;
import frc.robot.commands.Conveyor.*;
import frc.robot.commands.Grabber.CloseGrabber;
import frc.robot.commands.Grabber.OpenGrabber;
import frc.robot.commands.Grabber.OpenGrabberSearch;
import frc.robot.commands.Intake.ExtendIntake;
import frc.robot.commands.Intake.ManualExtendIntake;
import frc.robot.commands.Intake.ManualRetractIntake;
import frc.robot.commands.Intake.RetractIntake;
import frc.robot.commands.Lift.SetPosition0;
import frc.robot.commands.Lift.SetPosition1;
import frc.robot.commands.Lift.SetPosition2;
import frc.robot.commands.Lift.SetPosition3;
import frc.robot.commands.Lift.SetPositionIntake;
import frc.robot.commands.Swerve.AutoBalance;
import frc.robot.commands.Swerve.TeleopSwerve;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  
    /* Subsystems */
    //private final Joystick driver = new Joystick(0);
    private final XboxController driveController = new XboxController(0);
    private final XboxController operatorController = new XboxController(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driveController, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driveController, XboxController.Button.kLeftBumper.value);
    private final JoystickButton setZeroPoints = new JoystickButton(driveController, XboxController.Button.kRightBumper.value);

    /* Operator Buttons */
    //private final JoystickButton conveyorManualForward = new JoystickButton(operatorController, XboxController.Button.kB.value);
    //private final JoystickButton conveyorManualBackward = new JoystickButton(operatorController, XboxController.Button.kB.value);
    private final JoystickButton intakeRun = new JoystickButton(operatorController, XboxController.Button.kLeftBumper.value);
    //private final JoystickButton intakeManualRetract = new JoystickButton(driveController, XboxController.Button.kY.value);
    //private final JoystickButton intakeManualExtend = new JoystickButton(driveController, XboxController.Button.kA.value);
    private final JoystickButton SetLiftPosition0 = new JoystickButton(operatorController, XboxController.Button.kA.value);
    private final JoystickButton SetLiftPosition1 = new JoystickButton(operatorController, XboxController.Button.kX.value);
    private final JoystickButton SetLiftPosition2 = new JoystickButton(operatorController, XboxController.Button.kB.value);
    private final JoystickButton SetLiftPosition3 = new JoystickButton(operatorController, XboxController.Button.kY.value);
    private final JoystickButton SetLiftPositionIntake = new JoystickButton(operatorController, XboxController.Button.kRightBumper.value);
    private final Trigger GrabberDropCone = new Trigger(() -> operatorController.getRawAxis(3) > 0.5);

    private final Trigger bothPhotoEyesBlocked = new Trigger(() -> {
        return (Constants.Sensors.photoeye1.get() && Constants.Sensors.photoeye2.get());
    });

    private final Trigger onePhotoEyeBlocked = new Trigger(() -> {
      //One eye is blocked but both eyes are not
      return ((Constants.Sensors.photoeye1.get() || Constants.Sensors.photoeye2.get()) && (Constants.Sensors.photoeye1.get() && Constants.Sensors.photoeye2.get()));
    });

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Conveyor s_Conveyor = new Conveyor();
    private final Intake s_Intake = new Intake();
    private final Lift s_Lift = new Lift();
    private final Grabber s_Grabber = new Grabber();
    private final Limelight s_Limelight = new Limelight();

    /* Auto Commands */
    private final Mobility c_Mobility = new Mobility(s_Swerve);
    private final ScoreBalanceAdjacent c_ScoreBalanceAdjacent = new ScoreBalanceAdjacent(s_Swerve, s_Lift, s_Grabber);
    private final ScoreBalanceOpposite c_ScoreBalanceOpposite = new ScoreBalanceOpposite(s_Swerve, s_Lift, s_Grabber);
    private final Score2Adjacent c_Score2Adjacent = new Score2Adjacent(s_Swerve, s_Lift, s_Intake, s_Grabber);
    private final Score2Opposite c_Score2Opposite = new Score2Opposite(s_Swerve, s_Lift, s_Intake, s_Grabber);
    private final ScoreBalance c_ScoreBalance = new ScoreBalance(s_Swerve, s_Lift, s_Grabber);
    private final AutoBalance c_AutoBalance = new AutoBalance(s_Swerve);

    private final SendableChooser<Command> autonomousSelector = new SendableChooser<Command>();
    private final SendableChooser<String> modeSelector = new SendableChooser<String>();


    private SlewRateLimiter translationLimiter = new SlewRateLimiter(2);
    private SlewRateLimiter strafeLimiter = new SlewRateLimiter(2);
    private SlewRateLimiter rotationLimiter = new SlewRateLimiter(2);


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        //SET DEFAULT COMMANDS  
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> translationLimiter.calculate(-driveController.getRawAxis(translationAxis)), 
                () -> strafeLimiter.calculate(-driveController.getRawAxis(strafeAxis)), 
                () -> rotationLimiter.calculate(-driveController.getRawAxis(rotationAxis)), 
                () -> robotCentric.getAsBoolean()
            )
        );

        
        s_Conveyor.setDefaultCommand(new TurnOffConveyor(s_Conveyor));
        s_Intake.setDefaultCommand(new RetractIntake(s_Intake));
        s_Lift.setDefaultCommand(null);
        s_Grabber.setDefaultCommand(null);

        autonomousSelector.setDefaultOption("Mobility", c_Mobility);
        autonomousSelector.addOption("Score Balance Adjacent", c_ScoreBalanceAdjacent);
        autonomousSelector.addOption("Score Balance Opposite", c_ScoreBalanceOpposite);
        autonomousSelector.addOption("Score 2 Adjacent", c_Score2Adjacent);
        autonomousSelector.addOption("Score 2 Opposite", c_Score2Opposite);
        autonomousSelector.addOption("Score Balance", c_ScoreBalance);
        autonomousSelector.addOption("Test AutoBalance", c_AutoBalance);

        // Configure the button bindings
        configureButtonBindings();

        putDashboard();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        setZeroPoints.onTrue(new SetZeroPoints(s_Lift));
        
        intakeRun.whileTrue(new ParallelCommandGroup(new ExtendIntake(s_Intake, false), new TurnOnConveyor(s_Conveyor)));
        intakeRun.whileFalse(new ParallelCommandGroup(new RetractIntake(s_Intake), new TurnOffConveyor(s_Conveyor)));
        SetLiftPosition0.onTrue(new SequentialCommandGroup(new OpenGrabber(s_Grabber), new SetPosition0(s_Lift)));
        SetLiftPosition1.onTrue(new SequentialCommandGroup(new CloseGrabber(s_Grabber), new SetPosition1(s_Lift), new TurnOffConveyor(s_Conveyor)));
        SetLiftPosition2.onTrue(new SequentialCommandGroup(new CloseGrabber(s_Grabber), new SetPosition2(s_Lift), new TurnOffConveyor(s_Conveyor)));
        SetLiftPosition3.onTrue(new SequentialCommandGroup(new CloseGrabber(s_Grabber), new SetPosition3(s_Lift), new TurnOffConveyor(s_Conveyor)));
        SetLiftPositionIntake.onTrue(new SequentialCommandGroup(new OpenGrabber(s_Grabber), new SetPositionIntake(s_Lift), new TurnOffConveyor(s_Conveyor)));
        GrabberDropCone.onTrue(new OpenGrabberSearch(s_Grabber, s_Limelight.getX()));
        
        bothPhotoEyesBlocked.onTrue(new CloseGrabber(s_Grabber));
        onePhotoEyeBlocked.onTrue(new SequentialCommandGroup(new Delay(2.0), new ParallelCommandGroup(new CloseGrabber(s_Grabber), new TurnOffConveyor(s_Conveyor))));
        
        //Special Conditional Commands
    }
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
      return autonomousSelector.getSelected();
  }

  public void putDashboard(){
      SmartDashboard.putData("Autonomous Mode", autonomousSelector);
  }
}
