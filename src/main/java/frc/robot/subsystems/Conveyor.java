// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Conveyor extends SubsystemBase {
    /** Creates a new Conveyor. */
    private DigitalInput intakeSensor = new DigitalInput(Constants.CONVEYOR_INTAKE_SENSOR);
    private DigitalInput liftSensor = new DigitalInput(Constants.CONVEYOR_LIFT_SENSOR);
    private CANSparkMax conveyorMotor = new CANSparkMax(Constants.CONVEYOR_CONTROLLER, MotorType.kBrushless);
    private boolean inTransitionState = false;
    private boolean logging = false;

    public Conveyor() {
        conveyorMotor.clearFaults();
        conveyorMotor.restoreFactoryDefaults();
        conveyorMotor.setOpenLoopRampRate(Constants.CONVEYOR_RAMP_RATE);
    }

    public void setConveyor(double speed){
        if(intakeSensor.get() || inTransitionState){
            conveyorMotor.setVoltage(speed);
            inTransitionState = true;
        }
        else if(liftSensor.get()){
            conveyorMotor.stopMotor();
            inTransitionState = false;
        }
    }

    public void setConveyorManual(double speed){
      conveyorMotor.setVoltage(speed);
    }

    public boolean isConveyorLoaded(){
        if(intakeSensor.get() || liftSensor.get() || inTransitionState){
            return true;
        }
        return false;
    }

    public void changeLoggingState(boolean state){
        this.logging = state;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if(logging){
            SmartDashboard.putBoolean("Intake Sensor", intakeSensor.get());
            SmartDashboard.putBoolean("Lift Sensor", liftSensor.get());
            SmartDashboard.putBoolean("Transition State Flag", inTransitionState);
            SmartDashboard.putNumber("Conveyor Motor CAN ID", conveyorMotor.getDeviceId());
            SmartDashboard.putNumber("Conveyor Motor Set Speed", conveyorMotor.get());
            SmartDashboard.putNumber("Conveyor Motor Temperature (Celsius)", conveyorMotor.getMotorTemperature());
        }
    }
}
