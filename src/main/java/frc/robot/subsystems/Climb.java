// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.MotorConfigs;

public class Climb extends SubsystemBase {
  private Servo m_servo;
  private SparkMax m_pivot;
  private SparkClosedLoopController controller;
  private static Climb instance;
   
  /** Creates a new Climb. */
  public Climb() {
    m_servo = new Servo(Constants.ClimbConstants.servoID);
    m_pivot = new SparkMax(Constants.ClimbConstants.pivotID, MotorType.kBrushless);
    
    m_pivot.configure(MotorConfigs.climb_pivot_configs, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    controller = m_pivot.getClosedLoopController();
    // addRequirements();
  }

  public void setPivotSpeed(double speed) {
    m_pivot.set(speed);
  }

  public void setPivotPosition(double position){  
    controller.setReference(position, ControlType.kPosition);
    //controller.setReference(position, ControlType.kMAXMotionPositionControl);
  }

  

  public void setClawPosition(double position){ // position
    m_servo.set(position); // setPosition
    //m_servo.setSpeed(position);
    // m_servo.setSpeed(position);
  }

  public boolean isServoClosed(){
    return (m_servo.get() == 1.0) ? true : false;
  }
    

  public double getPivotPosition(){
    return m_pivot.getEncoder().getPosition();
  }

  public double getPivotVelocity(){
    return m_pivot.getEncoder().getVelocity();
  }

  public Servo getServoMotor(){
    return m_servo;
  }

  public double getServoPosition(){
    return m_servo.getPosition();
  }

  public SparkMax getClimbMotor(){
    return m_pivot;
  }

  public static Climb getInstance(){
    if (instance == null){
      instance = new Climb();
    }
    return instance;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}