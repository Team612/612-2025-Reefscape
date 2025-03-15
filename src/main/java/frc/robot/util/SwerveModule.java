// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.MotorConfigs;

public class SwerveModule extends SubsystemBase {
    private SparkMax angleMotor;
    private SparkMax driveMotor;
    private CANcoder angleEncoder;
    private double encoderOffset;
    private PIDController turnPIDController = new PIDController(Constants.SwerveConstants.anglekP, 0, 0);    

  public SwerveModule(int angleMotorID, int drivingMotorID, int angleEncoderID, double encoderOffset){
    angleMotor = new SparkMax(angleMotorID,MotorType.kBrushless);
    driveMotor = new SparkMax(drivingMotorID,MotorType.kBrushless);
    angleEncoder = new CANcoder(angleEncoderID);

    angleMotor.configure(MotorConfigs.swerve_angle_configs, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    driveMotor.configure(MotorConfigs.swerve_drive_configs, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    this.encoderOffset = encoderOffset;
    turnPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @SuppressWarnings("deprecation")
  public void setState(SwerveModuleState desiredState){
    SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, new Rotation2d(getCurrentAngle()));
    driveMotor.set(optimizedState.speedMetersPerSecond * Constants.SwerveConstants.metersPerSecondtoMotorPercentConstant);
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
    return driveMotor.getEncoder().getVelocity();
  }

  public SwerveModulePosition getCurrentWheelPosition(){
    return new SwerveModulePosition(driveMotor.getEncoder().getPosition(), new Rotation2d(getCurrentAngle()));
  }

  public void resetEncoder(){
    driveMotor.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {}
}
