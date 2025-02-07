// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Kraken extends SubsystemBase {
  /** Creates a new Kraken. */
  private TalonFX kraken;
  public Kraken(TalonFX kraken) {
    this.kraken = kraken;
  }
  public void setVelocity(double speed) {
    kraken.set(speed);
  }
  public void setVoltage(double volts) {
    kraken.setVoltage(volts);
  }
  public void stop() {
    kraken.stopMotor();
  }
  public double getVelocity() {
    return kraken.get();
  }
  public boolean motorAlive() {
    return kraken.isAlive();
  }
  public boolean safetyEnabled() {
    return kraken.isSafetyEnabled();
  }
  public void setSafety(boolean enabled) {
    kraken.setSafetyEnabled(enabled);
  }
  public double getVoltage() {
    return kraken.getMotorVoltage().getValueAsDouble();
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Kraken velocity", this.getVelocity());
    SmartDashboard.putBoolean("Kraken alive", this.motorAlive());
    SmartDashboard.putBoolean("Kraken Safety", this.safetyEnabled());
    // This method will be called once per scheduler run
  }
}
