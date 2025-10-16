// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PayloadConstants;

public class Payload extends SubsystemBase {
  private PIDController elevatorPID;
  private PIDController intakePID;

  private SparkMax elevatorMotor;
  private SparkMax pivotMotor;

  public Payload() {
    elevatorMotor = new SparkMax(PayloadConstants.elevatorID,MotorType.kBrushless);
    pivotMotor = new SparkMax(PayloadConstants.pivotID,MotorType.kBrushless);

    elevatorPID = new PIDController(PayloadConstants.elevatorkp, 0, 0);
    intakePID = new PIDController(PayloadConstants.intakekp,0,0);
  }

  public void setElevator(double speed){
    elevatorMotor.set(speed);
  }
  public void setIntake(double speed){
    pivotMotor.set(speed);
  }
  public double getElevatorPID(double setpoint){
    return elevatorPID.calculate(elevatorMotor.getEncoder().getPosition(),setpoint);
  }
  public double getIntakePID(double setpoint){
    double value = -intakePID.calculate(pivotMotor.getEncoder().getPosition(),setpoint);
    // if (value > 0)
    //   value += PayloadConstants.minPivotSpeed;
    // else
    //   value -= PayloadConstants.minPivotSpeed;
    return value;
  }
  public double getElevatorPos(){
    return elevatorMotor.getEncoder().getPosition();
  }
  public double getIntakePos(){
    return pivotMotor.getEncoder().getPosition();
  }
  public void setElevatorPos(double pos){
    elevatorMotor.getEncoder().setPosition(pos);
  }
  public void setIntakePos(double pos){
    pivotMotor.getEncoder().setPosition(pos);
  }

  @Override
  public void periodic() {
    if (elevatorMotor.getForwardLimitSwitch().isPressed())
      elevatorMotor.getEncoder().setPosition(0);
    if (elevatorMotor.getReverseLimitSwitch().isPressed()) 
      elevatorMotor.getEncoder().setPosition(PayloadConstants.topElevatorPosition);

    if (pivotMotor.getForwardLimitSwitch().isPressed())
      pivotMotor.getEncoder().setPosition(0);
    if (pivotMotor.getReverseLimitSwitch().isPressed())
      pivotMotor.getEncoder().setPosition(PayloadConstants.backIntakePosition);

    SmartDashboard.putNumber("ElevatorPos", elevatorMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("PivotPos", pivotMotor.getEncoder().getPosition());
  }
}
