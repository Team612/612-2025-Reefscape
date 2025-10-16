// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BagConstants;

public class Bag extends SubsystemBase {

  private SparkMax bagMotor;

  public Bag() {
    bagMotor = new SparkMax(BagConstants.bagID, MotorType.kBrushed);
  }

  public void setBag(double speed){
    bagMotor.set(speed);
  }

  @Override
  public void periodic() {}
}
