// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class mySwerveModule extends SubsystemBase {
    private SparkMax angleMotor;
    private SparkMax drivingMotor;
    private CANcoder angleEncoder;
    private double encoderOffset;
    private PIDController turnPIDController = new PIDController(Constants.kp, 0, 0);    

  public mySwerveModule(int angleMotorID, int drivingMotorID, int angleEncoderID, double encoderOffset){
    angleMotor = new SparkMax(angleMotorID,MotorType.kBrushless);
    drivingMotor = new SparkMax(drivingMotorID,MotorType.kBrushless);
    angleEncoder = new CANcoder(angleEncoderID);
    this.encoderOffset = encoderOffset;
    turnPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @SuppressWarnings("deprecation")
  public void setMySwerveState(SwerveModuleState desiredState){
    SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, new Rotation2d(getCurrentAngle()));
    drivingMotor.set(optimizedState.speedMetersPerSecond * Constants.metersPerSecondtoMotorPercentConstant);
    angleMotor.set(turnPIDController.calculate(getCurrentAngle(), optimizedState.angle.getRadians()));
  }

  public double getCurrentAngle() {
    double rotations = angleEncoder.getAbsolutePosition().getValueAsDouble() - encoderOffset;
    if (rotations < 0)
      rotations += 1;      
    if (rotations < 0.5)
      return rotations * 2 * Math.PI;
    else
      return (rotations-1) * 2 * Math.PI;
  }

  public double getCurrentVelocity(){
    return drivingMotor.getEncoder().getVelocity();
  }

  public SwerveModulePosition getCurrentWheelPosition(){
    return new SwerveModulePosition(drivingMotor.getEncoder().getPosition(), new Rotation2d(getCurrentAngle()));
  }

  public void resetEncoder(){
    drivingMotor.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {}
}
