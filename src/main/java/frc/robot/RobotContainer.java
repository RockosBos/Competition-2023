// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//This project utilizes a modified version of FIRST Robotics Competition Team 364's Open source Swerve Drive code. 
//This project can be accessed from the following link: https://github.com/Team364/BaseFalconSwerve

package frc.robot;

import frc.robot.commands.Autonomous.Mobility;
import frc.robot.commands.Autonomous.Score2Left;
import frc.robot.commands.Autonomous.Score2Right;
import frc.robot.commands.Autonomous.ScoreBalanceLeft;
import frc.robot.commands.Autonomous.ScoreBalanceRight;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Swerve;

import frc.robot.commands.Conveyor.*;
import frc.robot.commands.Swerve.AutoBalance;
import frc.robot.commands.Swerve.TeleopSwerve;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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

    /* Operator Buttons */
    private final JoystickButton conveyorManualForward = new JoystickButton(operatorController, XboxController.Button.kB.value);
    private final JoystickButton conveyorManualBackward = new JoystickButton(operatorController, XboxController.Button.kB.value);
    private final JoystickButton intakeRun = new JoystickButton(operatorController, XboxController.Button.kB.value);
    private final JoystickButton intakeManualRetract = new JoystickButton(operatorController, XboxController.Button.kB.value);
    private final JoystickButton intakeManualExtend = new JoystickButton(operatorController, XboxController.Button.kB.value);
    private final JoystickButton SetLiftPosition0 = new JoystickButton(operatorController, XboxController.Button.kB.value);
    private final JoystickButton SetLiftPosition1 = new JoystickButton(operatorController, XboxController.Button.kB.value);
    private final JoystickButton SetLiftPosition2 = new JoystickButton(operatorController, XboxController.Button.kB.value);
    private final JoystickButton SetLiftPosition3 = new JoystickButton(operatorController, XboxController.Button.kB.value);
    private final JoystickButton GrabberDropCube = new JoystickButton(operatorController, XboxController.Button.kB.value);
    private final JoystickButton GrabberDropCone = new JoystickButton(operatorController, XboxController.Button.kB.value);


    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    //private final Conveyor s_Conveyor = new Conveyor();
    //private final Intake s_Intake = new Intake();
    //private final Lift s_Lift = new Lift();

    /* Auto Commands */
    private final Mobility c_Mobility = new Mobility(s_Swerve);
    private final ScoreBalanceLeft c_ScoreBalanceLeft = new ScoreBalanceLeft(s_Swerve);
    private final ScoreBalanceRight c_ScoreBalanceRight = new ScoreBalanceRight(s_Swerve);
    private final Score2Left c_Score2Left = new Score2Left(s_Swerve);
    private final Score2Right c_Score2Right = new Score2Right(s_Swerve);
    private final AutoBalance c_AutoBalance = new AutoBalance(s_Swerve);

    private final SendableChooser<Command> autonomousSelector = new SendableChooser<Command>();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        //SET DEFAULT COMMANDS  
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driveController.getRawAxis(translationAxis), 
                () -> -driveController.getRawAxis(strafeAxis), 
                () -> -driveController.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

        /*
        s_Conveyor.setDefaultCommand(new SetConveyorDefault(s_Conveyor));
        s_Intake.setDefaultCommand(new RetractIntake(s_Intake));
        s_Lift.setDefaultCommand(new SetPosition0(s_Lift, s_Intake.isIntakeRetracted()));
        */


        autonomousSelector.setDefaultOption("Mobility", c_Mobility);
        autonomousSelector.addOption("Score Balance Left", c_ScoreBalanceLeft);
        autonomousSelector.addOption("Score Balance Right", c_ScoreBalanceRight);
        autonomousSelector.addOption("Score 2 Left", c_Score2Left);
        autonomousSelector.addOption("Score 2 right", c_Score2Right);
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
        
        /*
        conveyorManualBackward.onTrue(new SetConveyorManualBackward(s_Conveyor));
        conveyorManualForward.onTrue(new SetConveyorManualForward(s_Conveyor));
        intakeRun.onTrue(new ParallelCommandGroup(new ExtendIntake(s_Intake, s_Lift.isLiftRetracted()), new TurnOnConveyor(s_Conveyor)));
        intakeManualExtend.onTrue(new ManualExtendIntake(s_Intake, s_Lift.isLiftRetracted()));
        intakeManualRetract.onTrue(new ManualRetractIntake(s_Intake, s_Lift.isLiftRetracted()));
        SetLiftPosition0.onTrue(new SetPosition0(s_Lift, s_Intake.isIntakeRetracted()));
        SetLiftPosition1.onTrue(new SetPosition1(s_Lift, s_Intake.isIntakeRetracted()));
        SetLiftPosition2.onTrue(new SetPosition2(s_Lift, s_Intake.isIntakeRetracted()));
        SetLiftPosition3.onTrue(new SetPosition3(s_Lift, s_Intake.isIntakeRetracted()));
        */
    }
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
      return autonomousSelector.getSelected();
  }

  public void putDashboard(){
      SmartDashboard.putData("Autonomous Mode", autonomousSelector);
  }
}
