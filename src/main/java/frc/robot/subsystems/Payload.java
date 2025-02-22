// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.MotorConfigs;

// import com.studica.frc.AHRS;
// import com.studica.frc.AHRS.NavXComType;

public class Payload extends SubsystemBase {
  private SparkMax elevatorMotor;
  private SparkClosedLoopController controller;
  // private SparkMax elevator2 = new SparkMax(Constants.elevatorID2, MotorType.kBrushless);

  // private SparkMax neoPivot = new SparkMax(Constants.neoPivotID, MotorType.kBrushless);
  private static Payload payloadInstance;
  // DigitalInput toplimitSwitch = new DigitalInput(Constants.toplimitSwitchID);
  // DigitalInput bottomlimitSwitch = new DigitalInput(Constants.bottomlimitSwitchID);

  public Payload() {
    elevatorMotor = new SparkMax(Constants.ElevatorConstants.elevatorID, MotorType.kBrushless);
    elevatorMotor.configure(MotorConfigs.elevator_pivot_configs, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    controller = elevatorMotor.getClosedLoopController();
    Preferences.initDouble("Pay Speed", Constants.ElevatorConstants.payloadspeed);
    Preferences.initDouble("Elevator kI", Constants.ElevatorConstants.kI);
    Preferences.initDouble("Elevator kP", Constants.ElevatorConstants.kP);
    Preferences.initDouble("Elevator kD", Constants.ElevatorConstants.kD);
  }

public void setMotorSpeed(double speed) {
  elevatorMotor.set(speed);
}

public void setPosition(double position){
  controller.setReference(position, ControlType.kPosition);
  //controller.setReference(position, ControlType.kMAXMotionPositionControl);
}

public void freezeMotors(){
  elevatorMotor.set(0);
}

public void resetCount() {
  elevatorMotor.getEncoder().setPosition(0);
}

public double getPosition(){
  return elevatorMotor.getEncoder().getPosition();
}

public SparkMax getElevatorMotor() {
  return elevatorMotor;
}

public static Payload getInstance(){
  if (payloadInstance == null){
    payloadInstance = new Payload();
  }
  return payloadInstance;
}


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator 1 Velocity", elevatorMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Elevator 1 position", elevatorMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Elevator Speed (Controller)", Constants.ElevatorConstants.payloadspeed);
    // System.out.println(Constants.ElevatorConstants.payloadspeed);
    Constants.ElevatorConstants.payloadspeed = Preferences.getDouble("Pay Speed", Constants.ElevatorConstants.payloadspeed);
   
    // SmartDashboard.putNumber("Elevator 2 Velocity", elevator.getEncoder().getVelocity());
    // SmartDashboard.putNumber("Elevator 2 position", elevator.getEncoder().getPosition());
    Constants.ElevatorConstants.kP = Preferences.getDouble("Elevator Pivot kP", Constants.ElevatorConstants.kP);
    Constants.ElevatorConstants.kI = Preferences.getDouble("Elevator Pivot kI", Constants.ElevatorConstants.kI);
    Constants.ElevatorConstants.kD = Preferences.getDouble("Elevator Pivot kD", Constants.ElevatorConstants.kD);
  }
}