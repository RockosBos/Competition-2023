// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

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
    private boolean runConveyor = false;

    private boolean errorFlag;
    private String errorMessage;


    public Conveyor() {
        conveyorMotor.clearFaults();
        conveyorMotor.restoreFactoryDefaults();
        conveyorMotor.setOpenLoopRampRate(Constants.CONVEYOR_RAMP_RATE);
    }

    public void setConveyor(double voltage){
        if(!RobotContainer.photoEye.get()){
            conveyorMotor.setVoltage(voltage);
        }
        else{
            conveyorMotor.stopMotor();
            runConveyor = false;
        }
        
    }

    public void setConveyorManual(double speed){
      conveyorMotor.setVoltage(speed);
    }

    public void setConveyorState(boolean state){
        runConveyor = state;
    }

    public String getError(){
        return errorMessage;
    }

    public boolean getSensor(){
        return RobotContainer.photoEye.get();
    }

    public void clearErrors(){
        errorFlag = false;
        errorMessage = "";
    }

    @Override
    public void periodic() {
        if(runConveyor == true){
            setConveyor(Constants.CONVEYOR_FORWARD_SPEED_VOLTS);
        }
        else{
            conveyorMotor.stopMotor();
        }

        if(errorFlag){
            System.out.println(errorMessage);
        }
    }
}
