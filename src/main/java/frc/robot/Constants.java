package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;
    public static final double MAX_VOLTS = 12.0;

    //Motor ID's

    public static final int conveyorID = 15;
    public static final int intakeRollerID = 16;
    public static final int intakeExtendID = 17;
    public static final int liftRotateID = 18;
    public static final int liftExtendID = 19;
    public static final int grabberID = 20;
    public static final int extraMotor1ID = 21;
    public static final int extraMotor2ID = 22;

    //Sensor ID's

    public static final int conveyorPhotoEyeID = 0;
    public static final int intakeExtendLimitID = 1;
    public static final int intakeRetractLimitID = 2;
    public static final int liftZeroPositionLimitID = 3;

    //Speeds

    public static final double CONVEYOR_FORWARD_SPEED_VOLTS = MAX_VOLTS * 5.0;
    public static final double CONVEYOR_BACKWARD_SPEED_VOLTS = MAX_VOLTS * -5.0;
    public static final double INTAKE_EXTENTION_SPEED_VOLTS = MAX_VOLTS * 3.0;
    public static final double INTAKE_RETRACTION_SPEED_VOLTS = MAX_VOLTS * -3.0;
    public static final double INTAKE_ROLLER_SPEED_VOLTS = MAX_VOLTS * 6.0;
    public static final double LIFT_EXTEND_SPEED_VOLTS = MAX_VOLTS * 2.0;
    public static final double LIFT_ROTATE_SPEED_VOLTS = MAX_VOLTS * 2.0;

    //Motor Position

    public static final double LIFT_EXTEND_POSITION_0 = 0.0; //Conveyor Position
    public static final double LIFT_ROTATE_POSITION_0 = 0.0;
    public static final double LIFT_EXTEND_POSITION_1 = 0.0; //Level 1 Scoring Position
    public static final double LIFT_ROTATE_POSITION_1 = 0.0;
    public static final double LIFT_EXTEND_POSITION_2 = 0.0; //Level 2 Scoring Position
    public static final double LIFT_ROTATE_POSITION_2 = 0.0;
    public static final double LIFT_EXTEND_POSITION_3 = 0.0; //Level 3 Scoring Position
    public static final double LIFT_ROTATE_POSITION_3 = 0.0;
    public static final double LIFT_ROTATE_CLEAR_POSITION = 0.0; //Position where extension can be turned on.
    

    //Ramp Rates

    public static final double CONVEYOR_RAMP_RATE = 0.1;
    public static final double INTAKE_EXTEND_RAMP_RATE = 0.25;
    public static final double INTAKE_ROLLER_RAMP_RATE = 0.1;

    //Software Current Limits

    public static final double CONVEYOR_MAX_CURRENT = 40.0;
    public static final double INTAKE_EXTEND_MAX_CURRENT = 40.0;
    public static final double INTAKE_ROLLER_MAX_CURRENT = 40.0;

    public static final ShuffleboardTab matchTab = Shuffleboard.getTab("Match");
    public static final ShuffleboardTab swerveDebugTab = Shuffleboard.getTab("Swerve Debug");
    public static final ShuffleboardTab conveyorDebugTab = Shuffleboard.getTab("Conveyor Debug");
    public static final ShuffleboardTab intakeDebugTab = Shuffleboard.getTab("Intake Debug");
    public static final ShuffleboardTab liftDebugTab = Shuffleboard.getTab("Lift Debug");
    public static final ShuffleboardTab grabberDebugTab = Shuffleboard.getTab("Grabber Debug");
    public static final ShuffleboardTab pidConfigTab = Shuffleboard.getTab("PID Config");

    public static final class Swerve {
        public static final int pigeonID = 14;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants chosenModule = 
            COTSFalconSwerveConstants.SDSMK4(COTSFalconSwerveConstants.driveGearRatios.SDSMK4_L2);
        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(23.5); 
        public static final double wheelBase = Units.inchesToMeters(20.25); 
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.32 / 12); //TODO: This must be tuned to specific robot
        public static final double driveKV = (1.51 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class FrontLeftMod { 
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 6;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(273.3);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class FrontRightMod { 
            public static final int driveMotorID = 8;
            public static final int angleMotorID = 10;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(3.7);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class BackLeftMod { 
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(329.7);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class BackRightMod { 
            public static final int driveMotorID = 11;
            public static final int angleMotorID = 13;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(258.5);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 1;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}