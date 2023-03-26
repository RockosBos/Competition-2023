// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Conveyor;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Conveyor;

public class TurnOnConveyor extends CommandBase {
    private Conveyor s_Conveyor; 

    /** Creates a new Conveyor. */
    public TurnOnConveyor(Conveyor s_Conveyor) {
      this.s_Conveyor = s_Conveyor;
      addRequirements(this.s_Conveyor);
      // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      this.s_Conveyor.setServoPosition(Constants.CONVEYOR_SERVO_HIGH_POSITION);
      this.s_Conveyor.setConveyorState(true);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return this.s_Conveyor.getConveyorState();
    }
}
