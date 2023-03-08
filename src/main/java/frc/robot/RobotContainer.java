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
import frc.robot.commands.Autonomous.Mobility;
import frc.robot.commands.Autonomous.Score2Adjacent;
import frc.robot.commands.Autonomous.Score2Opposite;
import frc.robot.commands.Autonomous.ScoreBalanceAdjacent;
import frc.robot.commands.Autonomous.ScoreBalanceOpposite;
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

import edu.wpi.first.cameraserver.CameraServer;
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
    private final JoystickButton driverY_Button = new JoystickButton(driveController, XboxController.Button.kY.value);
    private final JoystickButton driverLB_Button = new JoystickButton(driveController, XboxController.Button.kLeftBumper.value);
    private final JoystickButton driverRB_Button = new JoystickButton(driveController, XboxController.Button.kRightBumper.value);

    /* Operator Buttons */
    //private final JoystickButton conveyorManualForward = new JoystickButton(operatorController, XboxController.Button.kB.value);
    //private final JoystickButton conveyorManualBackward = new JoystickButton(operatorController, XboxController.Button.kB.value);
    private final JoystickButton operatorLB_Button = new JoystickButton(operatorController, XboxController.Button.kLeftBumper.value);
    //private final JoystickButton intakeManualRetract = new JoystickButton(driveController, XboxController.Button.kY.value);
    //private final JoystickButton intakeManualExtend = new JoystickButton(driveController, XboxController.Button.kA.value);
    private final JoystickButton operatorA_Button = new JoystickButton(operatorController, XboxController.Button.kA.value);
    private final JoystickButton operatorX_Button = new JoystickButton(operatorController, XboxController.Button.kX.value);
    private final JoystickButton operatorB_Button = new JoystickButton(operatorController, XboxController.Button.kB.value);
    private final JoystickButton operatorY_Button = new JoystickButton(operatorController, XboxController.Button.kY.value);
    private final JoystickButton operatorRT_Button = new JoystickButton(operatorController, XboxController.Axis.kRightTrigger.value);
    private final JoystickButton operatorRB_Button = new JoystickButton(operatorController, XboxController.Button.kRightBumper.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Conveyor s_Conveyor = new Conveyor();
    private final Intake s_Intake = new Intake();
    private final Lift s_Lift = new Lift();
    private final Grabber s_Grabber = new Grabber();
    //private final Limelight s_Limelight = new Limelight();

    /* Auto Commands */
    private final Mobility c_Mobility = new Mobility(s_Swerve);
    private final ScoreBalanceAdjacent c_ScoreBalanceAdjacent = new ScoreBalanceAdjacent(s_Swerve, s_Lift, s_Grabber);
    private final ScoreBalanceOpposite c_ScoreBalanceOpposite = new ScoreBalanceOpposite(s_Swerve, s_Lift, s_Grabber);
    private final Score2Adjacent c_Score2Adjacent = new Score2Adjacent(s_Swerve, s_Lift, s_Intake, s_Grabber);
    private final Score2Opposite c_Score2Opposite = new Score2Opposite(s_Swerve, s_Lift, s_Intake, s_Grabber);
    private final AutoBalance c_AutoBalance = new AutoBalance(s_Swerve);

    private final SendableChooser<Command> autonomousSelector = new SendableChooser<Command>();
    public static final SendableChooser<String> modeSelector = new SendableChooser<String>();
    public static final DigitalInput photoEye = new DigitalInput(0);


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        CameraServer.startAutomaticCapture();

        //SET DEFAULT COMMANDS  
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driveController.getRawAxis(translationAxis), 
                () -> -driveController.getRawAxis(strafeAxis), 
                () -> -driveController.getRawAxis(rotationAxis), 
                () -> driverLB_Button.getAsBoolean()
            )
        );

        
        s_Conveyor.setDefaultCommand(new ConveyorOff(s_Conveyor));
        s_Intake.setDefaultCommand(new RetractIntake(s_Intake));
        s_Lift.setDefaultCommand(null);
        s_Grabber.setDefaultCommand(null);
        


        autonomousSelector.setDefaultOption("Mobility", c_Mobility);
        autonomousSelector.addOption("Score Balance Adjacent", c_ScoreBalanceAdjacent);
        autonomousSelector.addOption("Score Balance Opposite", c_ScoreBalanceOpposite);
        autonomousSelector.addOption("Score 2 Adjacent", c_Score2Adjacent);
        autonomousSelector.addOption("Score 2 Opposite", c_Score2Opposite);
        autonomousSelector.addOption("Test AutoBalance", c_AutoBalance);

        modeSelector.setDefaultOption("Auto", "Auto");
        modeSelector.addOption("Intakeless", "Intakeless");
        modeSelector.addOption("Manual", "Manual");

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
        driverY_Button.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

        //operatorLB_Button.onTrue(new ParallelCommandGroup(new ExtendIntake(s_Intake, s_Lift.isLiftRetracted()), new TurnOnConveyor(s_Conveyor)));
        operatorLB_Button.onTrue(new ParallelCommandGroup(new ExtendIntake(s_Intake, false), new ConveyorOn(s_Conveyor)));
        operatorY_Button.onTrue(new SequentialCommandGroup(new CloseGrabber(s_Grabber), new SetPosition0(s_Lift)));
        operatorX_Button.onTrue(new SequentialCommandGroup(new OpenGrabber(s_Grabber), new SetPosition1(s_Lift)));
        operatorB_Button.onTrue(new SequentialCommandGroup(new OpenGrabber(s_Grabber), new SetPosition2(s_Lift)));
        operatorA_Button.onTrue(new SequentialCommandGroup(new OpenGrabber(s_Grabber), new SetPosition3(s_Lift)));
        //operatorRB_Button.onTrue(new OpenGrabberSearch(s_Grabber, s_Limelight.getX()));
        operatorRT_Button.onTrue(new OpenGrabber(s_Grabber));

        //Manual Buttons

        //operatorLB_Button.onTrue();
        operatorY_Button.onTrue(new InstantCommand(() -> s_Lift.setManualRotateVoltage(Constants.LIFT_ROTATE_FORWARD_SPEED_VOLTS)));
        operatorY_Button.onFalse(new InstantCommand(() -> s_Lift.setManualRotateVoltage(0.0)));
        operatorA_Button.onTrue(new InstantCommand(() -> s_Lift.setManualRotateVoltage(Constants.LIFT_ROTATE_REVERSE_SPEED_VOLTS)));
        operatorA_Button.onFalse(new InstantCommand(() -> s_Lift.setManualRotateVoltage(0.0)));
        operatorB_Button.onTrue(new InstantCommand(() -> s_Lift.setManualExtendVoltage(Constants.LIFT_EXTEND_FORWARD_SPEED_VOLTS)));
        operatorB_Button.onFalse(new InstantCommand(() -> s_Lift.setManualExtendVoltage(0.0)));
        operatorX_Button.onTrue(new InstantCommand(() -> s_Lift.setManualExtendVoltage(Constants.LIFT_EXTEND_REVERSE_SPEED_VOLTS)));
        operatorX_Button.onFalse(new InstantCommand(() -> s_Lift.setManualExtendVoltage(0.0)));
        operatorRB_Button.onTrue(new InstantCommand(() -> s_Grabber.grabberManual(Constants.GRABBER_FORWARD_SPEED_VOLTS)));
        operatorRB_Button.onFalse(new InstantCommand(() -> s_Grabber.grabberManual(0.0)));
        operatorRT_Button.onTrue(new InstantCommand(() -> s_Grabber.grabberManual(Constants.GRABBER_REVERSE_SPEED_VOLTS)));
        operatorRT_Button.onFalse(new InstantCommand(() -> s_Grabber.grabberManual(0.0)));
        
    }
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return autonomousSelector.getSelected();
    }

    public void putDashboard(){
        SmartDashboard.putData("Autonomous Mode", autonomousSelector);
        SmartDashboard.putData("Control Mode", modeSelector);
    }
}
