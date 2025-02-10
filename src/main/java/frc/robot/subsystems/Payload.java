// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// import com.studica.frc.AHRS;
// import com.studica.frc.AHRS.NavXComType;

public class Payload extends SubsystemBase {
  private SparkMax elevator = new SparkMax(Constants.elevatorID, MotorType.kBrushless);
  // private SparkMax neoPivot = new SparkMax(Constants.neoPivotID, MotorType.kBrushless);
  private static Payload payloadInstance;
  DigitalInput toplimitSwitch = new DigitalInput(Constants.toplimitSwitchID);
  DigitalInput bottomlimitSwitch = new DigitalInput(Constants.bottomlimitSwitchID);

  public Payload() {
    
  }

public void setMotorSpeed(double speed) {
    if (speed > 0) {
        if (toplimitSwitch.get()) {
            elevator.set(0);
        } else {
            elevator.set(speed);
        }
    } else {
        if (bottomlimitSwitch.get()) {
            elevator.set(0);
        } else {
            elevator.set(speed);
        }
    }
}

  // public void pivot(){
  //   neoPivot.set(1.0);

  // }

  // public void depivot(){
  //   neoPivot.set(0.0);
  // }

  public double getPositiona() {
    return elevator.getEncoder().getPosition();
  }

  public static Payload getInstance(){
    if (payloadInstance == null){
      payloadInstance = new Payload();
    }
    return payloadInstance;
  }


  @Override
  public void periodic() {
    System.out.println(getInstance().getPositiona());
  }
}