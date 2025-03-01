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

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.MotorConfigs;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private SparkMax pivotMotor;
  private SparkClosedLoopController controller;
  private SparkMax bagMotor;
  private static Intake instance;


  double kDt = 0.02;
  double kMaxVelocity = 0.3;
  double kMaxAccel = 0.3;

  private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAccel);
  private final ProfiledPIDController m_controller = new ProfiledPIDController(Constants.IntakeConstants.kP, Constants.IntakeConstants.kI, Constants.IntakeConstants.kD, m_constraints);
  private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(Constants.IntakeConstants.kS, Constants.IntakeConstants.kG, Constants.IntakeConstants.kV);

  public Intake() {
    pivotMotor = new SparkMax(Constants.IntakeConstants.pivotID, MotorType.kBrushless);
    bagMotor = new SparkMax(Constants.IntakeConstants.bagID, MotorType.kBrushed);

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
    // controller.setReference(position, ControlType.kPosition);
    m_controller.setGoal(-position);
    pivotMotor.setVoltage(m_controller.calculate(getPivotPosition()) + m_feedforward.calculate(m_controller.getSetpoint().velocity));
    // controller.setReference(-position, ControlType.kMAXMotionPositionControl);
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
    return pivotMotor.getEncoder().getPosition() - Constants.IntakeConstants.boreOffset;
  }

  public double getPivotSpeed(){
    return pivotMotor.getEncoder().getVelocity();
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
    System.out.println("Pivot Pos: " + getPivotPosition());
    if (pivotMotor.getForwardLimitSwitch().isPressed()){
      pivotMotor.getEncoder().setPosition(0);
    }
  }
}
