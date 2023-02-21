// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import java.lang.Math;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class AutoBalance extends CommandBase {
  
    private Swerve s_Swerve;
    private double prevPitchSample;
    private double pitch;
    private double samplePeriod = 0.25;  //Frequency (in seconds) of checking samples
    private double sampleDifferenceThreshold = 3.0;   //Degree threshold that will determine if charging station is in motion
    private double balancedThreshold = 3.0;           //Degree threshold for the charge station to be considered threshold
    private double driveSpeed = 0.2;

    private Timer timer;

    private double translation;
    private double strafe;
    private double rotation;

    public AutoBalance(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.start();
        prevPitchSample = this.s_Swerve.getPitch();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        pitch = this.s_Swerve.getPitch();

        if(pitch > balancedThreshold){
          translation = driveSpeed;
        }
        else if(pitch < -balancedThreshold){
            translation = driveSpeed;
        }
        else{
            translation = 0.0;
        }

        if(timer.get() > samplePeriod){
            timer.reset();
            if(Math.abs(prevPitchSample - pitch) > sampleDifferenceThreshold){
                translation = 0.0;
                strafe = 0.0;
                rotation = 0.0;
            }
        }

        s_Swerve.drive(
            new Translation2d(translation, strafe).times(Constants.Swerve.maxSpeed),
            rotation * Constants.Swerve.maxAngularVelocity,
            false,
            true
        );

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
