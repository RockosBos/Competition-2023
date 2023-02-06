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

//This class will control the conveyor belt that serves the purpose of positioning cubes and cones so that they can be grabbed by the lift.

/*
 * Components *
 *  Conveyor Motor(14) - Spark Max Canbus powering Neo 550.
 *  Intake Photoeye(0) - Digital Input sensor located on the front of the robot.
 *  Lift Photoeye(1) - Digital Input sensor located on the back of the robot.
 * 
 * Description *
 *  This subsystem by default will not be controlled by the driver or operator (outside manual control).
 *  Instead the intake sensor will detect if a game piece has entered the conveyor and the conveyor will move until
 *  the game piece has reached the lift side sensor.
 * 
 * Commands *
 *  setConveyor (Default) - Runs the functionality described above.
 *  setManualForward - Manually runs the conveyor forward.
 *  setManualBackward - Manually runs the conveyor backward.
 * 
 * Functions *
 *  setConveyor - Turns on conveyor based on photoeye states.
 *  setConveyorManual - Turns on conveyor based on input speed.
 *  isconveyorLoaded - Returns state of the conveyor to find if it is loaded.
 */

public class Conveyor extends SubsystemBase {
    /** Creates a new Conveyor. */
    private DigitalInput intakeSensor = new DigitalInput(Constants.conveyorFrontPhotoEyeID);
    private DigitalInput liftSensor = new DigitalInput(Constants.conveyorBackPhotoEyeID);
    private CANSparkMax conveyorMotor = new CANSparkMax(Constants.conveyorID, MotorType.kBrushless);
    private boolean inTransitionState = false;

    private boolean logging = false;
    private boolean errorFlag;
    private String errorMessage;

    public Conveyor() {
        conveyorMotor.clearFaults();
        conveyorMotor.restoreFactoryDefaults();
        conveyorMotor.setOpenLoopRampRate(Constants.CONVEYOR_RAMP_RATE);
    }

    public void setConveyor(double speed){
        if(conveyorMotor.getOutputCurrent() < Constants.CONVEYOR_MAX_CURRENT){
            if(intakeSensor.get() || inTransitionState){
                conveyorMotor.setVoltage(speed);
                inTransitionState = true;
            }
            if(liftSensor.get()){
                conveyorMotor.stopMotor();
                inTransitionState = false;
            }
        }
        else{
            errorFlag = true;
            errorMessage = "Conveyor Motor (ID:15) has exceeded its max current draw";
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

    public String getError(){
        return errorMessage;
    }

    public void clearErrors(){
        errorFlag = false;
        errorMessage = "";
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

        if(errorFlag){
            System.out.println(errorMessage);
        }
    }
}
