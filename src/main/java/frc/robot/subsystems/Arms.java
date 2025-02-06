// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/***
 * VERY IMPORTANT:
 * The arms are controlled by a WINDOW motor.
 * This means that there is no encodor for it.
 * This will be controlled purely by the limit switches.
 */
package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arms extends SubsystemBase {
  /** Creates a new Arms. */
  private final SparkMax m_arms_motor;
  private SparkBaseConfig config;
  private ResetMode resetMode;
  private PersistMode persistMode;
  private LimitSwitchConfig limitSwitches;
  public Arms(SparkMax arms_motor) {
    this.m_arms_motor = arms_motor;
    config = new SparkMaxConfig();
    limitSwitches = new LimitSwitchConfig();
  }

  public boolean getForwardLimitSwitchPressed() {
    return m_arms_motor.getForwardLimitSwitch().isPressed();
  }
  public boolean getReverseLimitSwitchPressed() {
    return m_arms_motor.getReverseLimitSwitch().isPressed();
  }
  private void setConfig() {
    m_arms_motor.configure(config, resetMode, persistMode);
  }
  public void setForwardLimitSwitch(boolean enabled) { // disables motor shutdown if limitswitch is pressed (false enables)
    config.apply(limitSwitches.forwardLimitSwitchEnabled(enabled));
    this.setConfig();
  }
  public void setReverseLimitSwitch(boolean enabled) { // disables motor shutdown if limitswitch is pressed (false enables)
    config.apply(limitSwitches.reverseLimitSwitchEnabled(enabled));
    this.setConfig();
  }
  public void setForwardLimitSwitchType(boolean open) { // set whether or not the limit switch is normally open
    config.apply(limitSwitches.forwardLimitSwitchType(open ? Type.kNormallyOpen : Type.kNormallyClosed));
    this.setConfig();
  }
  public void setReverseLimitSwitchType(boolean open) { // set whether or not the limit switch is normally open
    config.apply(limitSwitches.forwardLimitSwitchType(open ? Type.kNormallyOpen : Type.kNormallyClosed));
    this.setConfig();
  }
  public void setVelocity(double speed) {
    m_arms_motor.set(speed);
  }
  public void setResetMode(boolean safe) {
    resetMode = safe ? ResetMode.kResetSafeParameters : ResetMode.kNoResetSafeParameters;
    this.setConfig();
  }
  public void setPersistMode(boolean parameters) {
    persistMode = parameters ? PersistMode.kPersistParameters : PersistMode.kNoPersistParameters;
    this.setConfig();
  }

  @Override
  public void periodic() {
    this.setPersistMode(true);
    SmartDashboard.putBoolean("Arms Forward Switch", this.getForwardLimitSwitchPressed());
    SmartDashboard.putBoolean("Arms Reverse Switch", this.getReverseLimitSwitchPressed());
    // This method will be called once per scheduler run
  }
}
