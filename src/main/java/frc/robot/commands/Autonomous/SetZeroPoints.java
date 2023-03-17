// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Lift;

public class SetZeroPoints extends CommandBase {
  /** Creates a new SetZeroPoints. */
  Lift s_Lift;
  boolean rotateSwitchHit, extendSwitchHit;

  public SetZeroPoints(Lift s_Lift){
    this.s_Lift = s_Lift;
    addRequirements(s_Lift);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotateSwitchHit = false;
    extendSwitchHit = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      s_Lift.setPositionControl(false);
      s_Lift.setExtendVoltage(Constants.LIFT_EXTEND_SPEED_VOLTS);

      if(!Constants.Sensors.liftExtendZero.get()){
          s_Lift.setRotateVoltage(Constants.LIFT_ROTATE_SPEED_VOLTS);
          s_Lift.setExtendVoltage(0.0);
          extendSwitchHit = true;
          s_Lift.zeroExtendEncoder();
          if(Constants.Sensors.liftRotateZero.get()){
            rotateSwitchHit = true;
            s_Lift.setRotateVoltage(0);
            s_Lift.zeroRotateEncoder();
          }
          else{
              s_Lift.setRotateVoltage(Constants.LIFT_ROTATE_SPEED_VOLTS);
          }
      }
      else{
          s_Lift.setRotateVoltage(0.0);
      }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Lift.setPositionControl(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(s_Lift.emergencyStop()){
      return true;
    }

    if(extendSwitchHit && rotateSwitchHit){
      s_Lift.setPositionControl(true);
      return true;
    }
    return false;
  }
}
