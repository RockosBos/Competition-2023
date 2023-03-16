// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LED;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LED;

public class SetLEDOnDriverStation extends CommandBase {
  private LED s_LED;
  
  public SetLEDOnDriverStation(LED s_LED) {
    this.s_LED = s_LED;
    addRequirements(this.s_LED);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(DriverStation.getAlliance() == Alliance.Red){
        s_LED.setStateRGB(255, 0, 0, "Solid");
    }
    else if(DriverStation.getAlliance() == Alliance.Blue){
        s_LED.setStateRGB(0, 0, 255, "Solid");
    }
    else{
        s_LED.setStateRGB(255, 255, 255, "Solid");
    }
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
