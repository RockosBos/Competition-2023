// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import java.lang.Math;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class AutoBalance extends CommandBase {
  
    private Swerve s_Swerve;
    private double prevRollSample;
    private double roll, prevRoll;
    private double samplePeriod = 0.25;  //Frequency (in seconds) of checking samples
    private double sampleDifferenceThreshold = 1;   //Degree threshold that will determine if charging station is in motion
    private double balancedThreshold = 2;           //Degree threshold for the charge station to be considered threshold
    private double driveSpeed = 0.2;

    private Timer timer = new Timer();

    private double translation;
    private double strafe;
    private double rotation;

    private boolean firstPass;

    public AutoBalance(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.start();
        prevRoll = this.s_Swerve.getRoll();
        firstPass = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        roll = this.s_Swerve.getRoll();
        
        /*
        if(timer.get() > 0.5){
            if(timer.get() < 0.75){
                if(roll > balancedThreshold){
                    translation = driveSpeed;
                }
                else if(roll < -balancedThreshold){
                    translation = -driveSpeed;
                }
                else{
                    timer.reset();
                    translation = 0.0;
                }
            }
            else{
                timer.reset();
            }
            
        }
        else{
            translation = 0.0;
        }

        prevRollSample = s_Swerve.gyro.getRoll();
        */
        if(timer.get() > 0.1){
            prevRoll = roll;
            timer.reset();
        }

        
        if(roll > 5.5){
            translation = 0.11;
        }
        else if(roll < -5.5){
            translation = -0.11;
        }
        else{
            firstPass = true;
            translation = 0.0;
        }
        if(!firstPass){
            translation *= 1.25;
        }
        /* 
        if(this.s_Swerve.getRoll() - prevRollSample > 0.5){
            translation = 0.15;
        }
        else if(this.s_Swerve.getRoll() - prevRollSample < -0.5){
            translation = -0.15;
        }
         */
        
        if(DriverStation.getMatchTime() < 0.1){
            translation = 0.0;
            strafe = 0.1;
        }
        else{
            strafe = 0;
        }
        
        s_Swerve.drive(
            new Translation2d(-translation, strafe).times(Constants.Swerve.maxSpeed),
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
