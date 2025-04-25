// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.subsystems.Payload;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.MotorConfigs;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private SparkMax pivotMotor;
  private SparkClosedLoopController controller;
  private SimpleMotorFeedforward m_feedForward;
  private static Intake instance;
  private DigitalInput m_limI = new DigitalInput(Constants.IntakeConstants.limIID);

  public Intake() {
    pivotMotor = new SparkMax(Constants.IntakeConstants.pivotID, MotorType.kBrushless);
    m_feedForward = new SimpleMotorFeedforward(0.0, 0.0, 0.0);

    pivotMotor.configure(MotorConfigs.spark_pivot_configs, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);    

    controller = pivotMotor.getClosedLoopController();
  }

  public void setPivotSpeed(double speed){
    if (!Payload.getSafetySwitch().get() && speed < 0){
      pivotMotor.set(0);
    }
    else {
    pivotMotor.set(speed);
    }
  }

  public void setPivotPosition(double position) {
    if (Payload.getSafetySwitch().get() && position < pivotMotor.getEncoder().getPosition()){
      pivotMotor.set(0);
    }
    else {
    controller.setReference(-position, ControlType.kPosition);
    }
    // controller.setReference(position, ControlType.kPosition,ClosedLoopSlot.kSlot0,m_feedForward.calculate(Constants.IntakeConstants.maxVelocity));
  }
  
  public boolean getIntakeLimitStateForward(){
    return pivotMotor.getForwardLimitSwitch().isPressed();
  }

  public boolean getIntakeLimitStateReverse(){
    return pivotMotor.getReverseLimitSwitch().isPressed();
  }

  public SparkMax getPivot(){
    return pivotMotor;
  }

  public double getPivotPosition(){
    return -pivotMotor.getEncoder().getPosition();
  }

  public double getPivotSpeed(){
    return -pivotMotor.getEncoder().getVelocity();
  }

  public static Intake getInstance(){
    if (instance == null){
      instance = new Intake();
    }
    return instance;
  }

  @Override
  public void periodic() {
    if (getIntakeLimitStateForward()){
      pivotMotor.getEncoder().setPosition(0.0);
    }

    SmartDashboard.putBoolean("Intake Limit Switch (Value): ", m_limI.get());
    //System.out.println(bagMotor.getEncoder().getPosition());
  }
}
