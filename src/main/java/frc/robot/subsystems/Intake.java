// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private SparkMax pivot;
  private TalonFX bagMotors;
  private static Intake instance;

  public Intake() {
    pivot = new SparkMax(0, MotorType.kBrushless);
    bagMotors = new TalonFX(0);

    SparkMaxConfig sp = new SparkMaxConfig();
    TalonFXConfiguration tp = new TalonFXConfiguration();
    tp.CurrentLimits.SupplyCurrentLimitEnable = true;
    tp.CurrentLimits.SupplyCurrentLimit = 30;
    sp.smartCurrentLimit(30);
    sp.idleMode(IdleMode.kBrake);
    pivot.configure(sp.inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    bagMotors.getConfigurator().apply(tp);
    
  }

  public boolean getIntakeLimitStateForward(){
    return pivot.getForwardLimitSwitch().isPressed();
  }

  // return limit switch states
  public boolean getIntakeLimitStateReverse(){
    return pivot.getReverseLimitSwitch().isPressed();
  }

  public SparkMax getPivot(){
    return pivot;
  }
  public TalonFX getBags(){
    return bagMotors;
  }
  public void setPivotSpeed(double speed){
      if (speed > 0) {
        if (pivot.getForwardLimitSwitch().isPressed()){// || (pivot.getEncoder().getPosition()) >= 0.5) {
            pivot.set(0);
            // elevator2.set(0);

        } else {
            pivot.set(speed);
            // elevator2.set(speed);
        }
    } else {
        if (pivot.getReverseLimitSwitch().isPressed()){// || (pivot.getEncoder().getPosition()) <= 0) {
            pivot.set(0);
            // elevator2.set(0);
        } else {
            pivot.set(speed);
            // elevator2.set(speed);
        }
    }
  }

  public double getPivotPosition(){
    return pivot.getEncoder().getPosition() - Constants.boreOffset;
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("speed", 0);
    SmartDashboard.getNumber("speed", 0);
    // This method will be called once per scheduler run
  }
}
