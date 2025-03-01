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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.MotorConfigs;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private SparkMax pivotMotor;
  private SparkClosedLoopController controller;
  private SparkMax bagMotor;
  private SimpleMotorFeedforward m_feedForward;
  private static Intake instance;

  public Intake() {
    pivotMotor = new SparkMax(Constants.IntakeConstants.pivotID, MotorType.kBrushless);
    bagMotor = new SparkMax(Constants.IntakeConstants.bagID, MotorType.kBrushed);
    m_feedForward = new SimpleMotorFeedforward(0.0, 0.0, 0.0);

    pivotMotor.configure(MotorConfigs.spark_pivot_configs, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    bagMotor.configure(MotorConfigs.spark_bag_configs, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    

    controller = pivotMotor.getClosedLoopController();
  }


  public void setBags(double speed){
    bagMotor.set(speed);
  }

  public void setPivotSpeed(double speed){
    pivotMotor.set(speed);
  }

  public void setPivotPosition(double position) {

    controller.setReference(-position, ControlType.kPosition);
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
  public SparkMax getBags(){
    return bagMotor;
  }

  public double getPivotPosition(){
    return -pivotMotor.getEncoder().getPosition();
  }

  public double getPivotSpeed(){
    return -pivotMotor.getEncoder().getVelocity();
  }

  public double getBagSpeed(){
    return bagMotor.get();
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
    //System.out.println(bagMotor.getEncoder().getPosition());
  }
}
