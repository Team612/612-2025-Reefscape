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

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private SparkMax elevator;
  private SparkBaseConfig config;
  private EncoderConfig encoderConfig;
  public Elevator(SparkMax elevator) {
    this.elevator = elevator;
    config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);
    encoderConfig.velocityConversionFactor(Constants.ElevatorConversionFactor);
  }
  public double getVelocity() {
    return elevator.getAbsoluteEncoder().getVelocity();
  }
  public void setVelocity(double speed) {
    elevator.set(speed);
  }
  public double getPosition() {
    return elevator.getAbsoluteEncoder().getPosition();
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Velocity", this.getVelocity());
    SmartDashboard.putNumber("Elevator position", this.getPosition());
    // This method will be called once per scheduler run
  }
}
