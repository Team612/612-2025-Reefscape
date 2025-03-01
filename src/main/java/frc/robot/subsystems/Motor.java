// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Motor extends SubsystemBase {
  /** Creates a new Motor. */
  private TalonSRX motor;
  // private static int counter = 0;
  public Motor(TalonSRX motor) {
    this.motor = motor;
  }
  // public double getVelocity() {
  //   return motor.getActiveTrajectoryVelocity();
  // }
  public void setVelocity(double speed) {
    motor.set(TalonSRXControlMode.Velocity, speed);
  }
  // public double getPosition() {
  //   return motor.getActiveTrajectoryPosition();
  // }
  // private double getSelectedSensorVelocity() {
  //   return motor.getSelectedSensorVelocity();
  // }
  // private double getSelectedSensorPosition() {
  //   return motor.getSelectedSensorPosition();
  // }
  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Active trajectory pos " + counter, this.getPosition());
    // SmartDashboard.putNumber("Active trajectory velocity " + counter, this.getVelocity());
    // SmartDashboard.putNumber("Selected trajectory pos " + counter, this.getSelectedSensorPosition());
    // SmartDashboard.putNumber("Selected trajectory velocity " + counter, this.getSelectedSensorVelocity());
    // This method will be called once per scheduler run
  }
}
