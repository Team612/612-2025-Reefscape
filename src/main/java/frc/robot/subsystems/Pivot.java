// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig;
// import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class Pivot extends SubsystemBase {
  /** Creates a new Pivot. */
  private final SparkMax m_pivot;
  private final SparkBaseConfig m_config;
  private ResetMode resetMode;
  private PersistMode persistMode;
  private LimitSwitchConfig limitSwitches;
  public Pivot(SparkMax pivot) {
    this.m_pivot = pivot;
    this.m_config = new SparkMaxConfig();
    limitSwitches = new LimitSwitchConfig();
  }

  public double getVelocity() {
    return m_pivot.getAbsoluteEncoder().getVelocity();
  }

  public boolean getForwardLimitSwitchPressed() {
    return m_pivot.getForwardLimitSwitch().isPressed();
  }
  public boolean getReverseLimitSwitchPressed() {
    return m_pivot.getReverseLimitSwitch().isPressed();
  }
  private void setConfig(SparkBaseConfig config) {
    m_pivot.configure(config, resetMode, persistMode);
  }
  public void setForwardLimitSwitch(boolean enabled) {
    SparkBaseConfig config = m_config.apply(limitSwitches.forwardLimitSwitchEnabled(enabled));
    this.setConfig(config);
  }
  public void setReverseLimitSwitch(boolean enabled) {
    SparkBaseConfig config = m_config.apply(limitSwitches.reverseLimitSwitchEnabled(enabled));
    this.setConfig(config);
  }
  public void setForwardLimitSwitchType(boolean open) {
    SparkBaseConfig config = m_config.apply(limitSwitches.forwardLimitSwitchType(open ? Type.kNormallyOpen : Type.kNormallyClosed));
    this.setConfig(config);
  }
  public void setReverseLimitSwitchType(boolean open) {
    SparkBaseConfig config = m_config.apply(limitSwitches.forwardLimitSwitchType(open ? Type.kNormallyOpen : Type.kNormallyClosed));
    this.setConfig(config);
  }
  public void setVelocity(double speed) {
    m_pivot.set(speed);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
