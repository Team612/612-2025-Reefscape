// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.MotorConfigs;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private SparkMax pivotMotor;
  private SparkClosedLoopController controller;
  private TalonFX bagMotor;
  private static Intake instance;

  public Intake() {
    pivotMotor = new SparkMax(Constants.IntakeConstants.pivotID, MotorType.kBrushless);
    bagMotor = new TalonFX(Constants.IntakeConstants.bagID);

    pivotMotor.configure(MotorConfigs.spark_pivot_configs, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    bagMotor.getConfigurator().apply(MotorConfigs.talon_bag_configs);

    controller = pivotMotor.getClosedLoopController();

    Preferences.initDouble("Pivot Speed", Constants.IntakeConstants.pivotspeed);
    Preferences.initDouble("Bag Speed", Constants.IntakeConstants.bagspeed);
    Preferences.initDouble("Intake Pivot kI", Constants.IntakeConstants.kI);
    Preferences.initDouble("Intake Pivot kP", Constants.IntakeConstants.kP);
    Preferences.initDouble("Intake Pivot kD", Constants.IntakeConstants.kD);
  }


  public void setBags(double speed){
    bagMotor.set(speed);
  }

  public void setPivotSpeed(double speed){
    pivotMotor.set(speed);
  }

  public void setPivotPosition(double position) {
    controller.setReference(position, ControlType.kPosition);
  }


  
  public boolean getIntakeLimitStateForward(){
    return pivotMotor.getForwardLimitSwitch().isPressed();
  }

  // return limit switch states
  public boolean getIntakeLimitStateReverse(){
    return pivotMotor.getReverseLimitSwitch().isPressed();
  }

  public SparkMax getPivot(){
    return pivotMotor;
  }
  public TalonFX getBags(){
    return bagMotor;
  }

  public double getPivotPosition(){
    return pivotMotor.getEncoder().getPosition() - Constants.IntakeConstants.boreOffset;
  }

  public static Intake getInstance(){
    if (instance == null){
      instance = new Intake();
    }
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Constants.IntakeConstants.pivotspeed = Preferences.getDouble("Pivot Speed", Constants.IntakeConstants.pivotspeed);
    Constants.IntakeConstants.bagspeed = Preferences.getDouble("Bag Speed", Constants.IntakeConstants.bagspeed);
    Constants.IntakeConstants.kP = Preferences.getDouble("Intake Pivot kP", Constants.IntakeConstants.kP);
    Constants.IntakeConstants.kI = Preferences.getDouble("Intake Pivot kI", Constants.IntakeConstants.kI);
    Constants.IntakeConstants.kD = Preferences.getDouble("Intake Pivot kD", Constants.IntakeConstants.kD);
  }
}
