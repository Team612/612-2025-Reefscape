// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shoot extends SubsystemBase { // SHOOT IS ACTUALLY NOT FOR SHOOTING, ITS FOR PIVOTING
  private SparkMax motor;
  /** Creates a new Shoot. */
  public Shoot(SparkMax motor) {
    this.motor = motor;
  }

  public void set(double speed) {
    motor.set(speed);
  }

  public double getVelocity() {
    return motor.getAbsoluteEncoder().getVelocity();
  }

  public double getPosition() {
    return motor.getAbsoluteEncoder().getPosition();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
