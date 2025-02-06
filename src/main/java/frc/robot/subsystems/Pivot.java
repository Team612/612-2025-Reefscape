// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private SparkMax m_pivot;
  private SparkBaseConfig config;
  private ResetMode resetMode;
  private PersistMode persistMode;
  private LimitSwitchConfig limitSwitches;
  public Pivot(SparkMax pivot) {
    this.m_pivot = pivot;
    config = new SparkMaxConfig();
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
  private void setConfig() {
    m_pivot.configure(config, resetMode, persistMode);
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
    m_pivot.set(speed);
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
    SmartDashboard.putNumber("Pivot Velocity", this.getVelocity());
    SmartDashboard.putBoolean("Pivot Forward Switch", this.getForwardLimitSwitchPressed());
    SmartDashboard.putBoolean("Pivot Reverse Switch", this.getReverseLimitSwitchPressed());
    // This method will be called once per scheduler run
  }
}
