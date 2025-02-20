// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.MotorConfigs;

// import com.studica.frc.AHRS;
// import com.studica.frc.AHRS.NavXComType;

public class Payload extends SubsystemBase {
  private SparkMax elevatorMotor = new SparkMax(Constants.ElevatorConstants.elevatorID, MotorType.kBrushless);
  // private SparkMax elevator2 = new SparkMax(Constants.elevatorID2, MotorType.kBrushless);

  // private SparkMax neoPivot = new SparkMax(Constants.neoPivotID, MotorType.kBrushless);
  private static Payload payloadInstance;
  // DigitalInput toplimitSwitch = new DigitalInput(Constants.toplimitSwitchID);
  // DigitalInput bottomlimitSwitch = new DigitalInput(Constants.bottomlimitSwitchID);

  public Payload() {
    elevatorMotor.configure(MotorConfigs.elevator_pivot_configs, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    Preferences.initDouble("Pay Speed", Constants.ElevatorConstants.payloadspeed);
  }

public void setMotorSpeed(double speed) {
  elevatorMotor.set(speed);
}

public void freezeMotors(){
  elevatorMotor.set(0);
  // elevator2.set(0);
}

  // public void pivot(){
  //   neoPivot.set(1.0);

  // }

  // public void depivot(){
  //   neoPivot.set(0.0);
  // }
  public void resetCount() {
    elevatorMotor.getEncoder().setPosition(0);
    // elevator2.getEncoder().setPosition(0);
  }
  
  // public double[] getPositiona() {
  //   double[] positions = new double[2];
  //   positions[0] = elevator.getEncoder().getPosition();
  //   positions[1] = elevator2.getEncoder().getPosition();
  //   return positions;
  // }
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
  }
}