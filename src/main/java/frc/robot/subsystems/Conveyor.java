// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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
    private CANSparkMax conveyorMotor = new CANSparkMax(Constants.conveyorID, MotorType.kBrushless);
    private Servo endStopServo = new Servo(0);
    private boolean runConveyor = false, runConveyorReverse = false;
    private double servoPosition;
    private GenericEntry sensorEntry1, sensorEntry2, sensorEntryValid1, sensorEntryValid2;

    private boolean errorFlag;
    private String errorMessage;

    private Timer photoeye1ValidityTimer = new Timer();
    private Timer photoeye2ValidityTimer = new Timer();
    private Timer stopConveyorDelay = new Timer();


    public Conveyor() {
        conveyorMotor.clearFaults();
        conveyorMotor.restoreFactoryDefaults();
        conveyorMotor.setInverted(true);
        conveyorMotor.setOpenLoopRampRate(Constants.CONVEYOR_RAMP_RATE);

        servoPosition = Constants.CONVEYOR_SERVO_HIGH_POSITION;

        sensorEntry1 = Constants.conveyorDebugTab.add("Photoeye 1", Constants.Sensors.photoeye1.get()).getEntry();
        sensorEntry2 = Constants.conveyorDebugTab.add("Photoeye 2", Constants.Sensors.photoeye2.get()).getEntry();
        sensorEntryValid1 = Constants.conveyorDebugTab.add("Photoeye 1 Valid", Constants.Sensors.photoeye1.get()).getEntry();
        sensorEntryValid2 = Constants.conveyorDebugTab.add("Photoeye 2 Valid", Constants.Sensors.photoeye2.get()).getEntry();
        photoeye1ValidityTimer.start();
        photoeye2ValidityTimer.start();
        stopConveyorDelay.start();
    }

    public boolean getConveyorState(){
        return runConveyor;
    }

    public void setConveyorManual(double speed){
      conveyorMotor.setVoltage(speed);
    }

    public void setConveyorState(boolean state){
        runConveyor = state;
        runConveyorReverse = false;
    }

    public void setconveyorReverseState(boolean state){
        runConveyorReverse = state;
        runConveyor = false;
    }

    public String getError(){
        return errorMessage;
    }

    public boolean getSensor(){
        return Constants.Sensors.photoeye1.get();
    }

    public void clearErrors(){
        errorFlag = false;
        errorMessage = "";
    }

    public boolean photoEye1BlockedValid(){
        if(Constants.Sensors.photoeye1.get()){
            if(photoeye1ValidityTimer.get() > 0.1){
                return true;
            }
        }
        else{
            photoeye1ValidityTimer.reset();
        }
        return false;
    }

    public boolean photoEye2BlockedValid(){
        if(Constants.Sensors.photoeye2.get()){
            if(photoeye2ValidityTimer.get() > 0.1){
                return true;
            }
        }
        else{
            photoeye2ValidityTimer.reset();
        }
        return false;
    }

    public boolean stopConveyorDelay(){
        if(photoEye1BlockedValid()){
            if(stopConveyorDelay.get() > 3.0){
                return true;
            }
        }
        else{
            stopConveyorDelay.reset();
        }
        return false;
    }

    public void setServoPosition(double pos){
        //Sets Servo Position in Degrees
        servoPosition = pos;
    }

    public double getServoPosition(){
        return servoPosition;
    }


    @Override
    public void periodic() {
        if(runConveyor){
            conveyorMotor.setVoltage(Constants.CONVEYOR_FORWARD_SPEED_VOLTS);
        }
        else if(runConveyorReverse){
            conveyorMotor.setVoltage(Constants.CONVEYOR_BACKWARD_SPEED_VOLTS);
        }
        else{
            conveyorMotor.setVoltage(0.0);
        }

        sensorEntry1.setBoolean(Constants.Sensors.photoeye1.get());
        sensorEntry2.setBoolean(Constants.Sensors.photoeye2.get());
        sensorEntryValid1.setBoolean(photoEye1BlockedValid());
        sensorEntryValid2.setBoolean(photoEye2BlockedValid());

        endStopServo.setAngle(servoPosition);

        if(errorFlag){
            System.out.println(errorMessage);
        }
    }
}
