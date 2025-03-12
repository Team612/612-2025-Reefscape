// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import frc.robot.Constants;
import frc.robot.util.SwerveModuleConstants;
import frc.robot.subsystems.Swerve;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
// import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
/** Add your docs here. */

public class SwerveModule {
    private SparkMax driveMotor;
    private SparkClosedLoopController driveController;
    private RelativeEncoder driveEncoder;
    private SimpleMotorFeedforward driveFeedforward;

    private SparkMax angleMotor;
    private SparkClosedLoopController angleController;
    private RelativeEncoder angleEncoder;

    private CANcoder canCoder;
    private double canCoderOffsets;
    
    public int moduleNumber;
    private double lastAngle;
    private Rotation2d zeroAngle;

    public SwerveModule(int moduleNumber, SwerveModuleConstants constants, SparkConfigs configs){
        this.moduleNumber = moduleNumber;

        driveMotor = new SparkMax(constants.driveMotorID, MotorType.kBrushless);
        driveController = driveMotor.getClosedLoopController();
        driveEncoder = driveMotor.getEncoder();
        driveFeedforward = new SimpleMotorFeedforward(Constants.SwerveConstants.driveKS, Constants.SwerveConstants.driveKV, Constants.SwerveConstants.driveKA);

        angleMotor = new SparkMax(constants.angleMotorID, MotorType.kBrushless);
        angleController = angleMotor.getClosedLoopController();
        angleEncoder = angleMotor.getEncoder();

        canCoder = new CANcoder(constants.cancoderID);
        canCoderOffsets = constants.desiredAngle.getDegrees();

        zeroAngle = constants.desiredAngle;
        configure(configs);
        lastAngle = getState().angle.getDegrees();
    }

    public void setState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState.optimize(getState().angle);

        if (isOpenLoop){
            double speed = desiredState.speedMetersPerSecond / Constants.SwerveConstants.maxSpeed;
            driveController.setReference(speed, ControlType.kDutyCycle);
        }
        else {
            driveController.setReference(desiredState.speedMetersPerSecond, ControlType.kVelocity, ClosedLoopSlot.kSlot0, driveFeedforward.calculate(desiredState.speedMetersPerSecond));
        }

        //prevents jittering
        double angle = Math.abs(desiredState.speedMetersPerSecond) <= Constants.SwerveConstants.maxSpeed * 0.01
        ? lastAngle
        : desiredState.angle.getDegrees();
  
        angleController.setReference(angle, ControlType.kPosition);
        lastAngle = angle;
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(angleEncoder.getPosition());
    }

    public SwerveModuleState getState(){
        double velocity = driveEncoder.getVelocity();
        Rotation2d rotation = new Rotation2d(Units.degreesToRadians(angleEncoder.getPosition()));

        return new SwerveModuleState(velocity, rotation);
    }

    public void resetToAbsolute(){
        //difference between the current absolute position and where zero is defined
        double absolutePosition = (canCoder.getAbsolutePosition().getValueAsDouble() * 360.0) - zeroAngle.getDegrees();

        angleEncoder.setPosition(absolutePosition);
       // angleController.setReference(absolutePosition, ControlType.kPosition);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveMotor.getEncoder().getPosition(), getAngle());
    }

    public void configure(SparkConfigs configs){
        driveMotor.configure(configs.driveMotorConfigs, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        angleMotor.configure(configs.angleMotorConfigs, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        driveEncoder.setPosition(0);
        resetToAbsolute();
   
        CANcoderConfiguration canCoderConfigs = new CANcoderConfiguration();
        canCoderConfigs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1; //unsigned [0,1]
        canCoderConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        
        canCoder.getConfigurator().apply(canCoderConfigs);
        
    }

    
    
}