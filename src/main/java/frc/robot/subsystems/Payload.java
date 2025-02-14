// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// import com.studica.frc.AHRS;
// import com.studica.frc.AHRS.NavXComType;

public class Payload extends SubsystemBase {
  private SparkMax elevator = new SparkMax(Constants.elevatorID, MotorType.kBrushless);
  // private SparkMax elevator2 = new SparkMax(Constants.elevatorID2, MotorType.kBrushless);

  // private SparkMax neoPivot = new SparkMax(Constants.neoPivotID, MotorType.kBrushless);
  private static Payload payloadInstance;
  DigitalInput toplimitSwitch = new DigitalInput(Constants.toplimitSwitchID);
  DigitalInput bottomlimitSwitch = new DigitalInput(Constants.bottomlimitSwitchID);

  public Payload() {
     SparkMaxConfig sp = new SparkMaxConfig();
    sp.smartCurrentLimit(30);
    sp.idleMode(IdleMode.kBrake);
    elevator.configure(sp.inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

public void setMotorSpeed(double speed) {
    if (speed > 0) {
        if (toplimitSwitch.get()) {
            elevator.set(0);
            // elevator2.set(0);

        } else {
            elevator.set(speed);
            // elevator2.set(speed);
        }
    } else {
        if (bottomlimitSwitch.get()) {
            elevator.set(0);
            // elevator2.set(0);
        } else {
            elevator.set(speed);
            // elevator2.set(speed);
        }
    }
}

  // public void pivot(){
  //   neoPivot.set(1.0);

  // }

  // public void depivot(){
  //   neoPivot.set(0.0);
  // }
  public void resetCount() {
    elevator.getEncoder().setPosition(0);
    // elevator2.getEncoder().setPosition(0);
  }
  
  // public double[] getPositiona() {
  //   double[] positions = new double[2];
  //   positions[0] = elevator.getEncoder().getPosition();
  //   positions[1] = elevator2.getEncoder().getPosition();
  //   return positions;
  // }

  public static Payload getInstance(){
    if (payloadInstance == null){
      payloadInstance = new Payload();
    }
    return payloadInstance;
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator 1 Velocity", elevator.getEncoder().getVelocity());
    SmartDashboard.putNumber("Elevator 1 position", elevator.getEncoder().getPosition());
    SmartDashboard.putNumber("Elevator Speed (Controller)", Constants.payspeed);
    System.out.println(Constants.payspeed);
    // SmartDashboard.putNumber("Elevator 2 Velocity", elevator.getEncoder().getVelocity());
    // SmartDashboard.putNumber("Elevator 2 position", elevator.getEncoder().getPosition());
  }
}