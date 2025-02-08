// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Motor extends SubsystemBase {
  /** Creates a new Motor. */
  private SparkMax motor;
  private SparkBaseConfig config;
  private EncoderConfig encoderConfig;
  public Motor(SparkMax motor) {
    this.motor = motor;
    config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);
    encoderConfig.velocityConversionFactor(Constants.MotorConversionFactor);
  }
  public double getVelocity() {
    return motor.getAbsoluteEncoder().getVelocity();
  }
  public void setVelocity(double speed) {
    motor.set(speed);
  }
  public double getPosition() {
    return motor.getAbsoluteEncoder().getPosition();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
