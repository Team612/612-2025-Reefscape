// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import frc.robot.Constants;

/** Add your docs here. */
public class MotorConfigs {
    public SparkMaxConfig driveMotorConfigs;
    public SparkMaxConfig angleMotorConfigs;
    CANcoderConfiguration canCoderConfigs;

    public MotorConfigs(){
        driveMotorConfigs = new SparkMaxConfig();
        angleMotorConfigs = new SparkMaxConfig();
        canCoderConfigs = new CANcoderConfiguration();
   
        
        driveMotorConfigs
            .inverted(Constants.driveInvert)
            .idleMode(Constants.driveNeutralMode)
            .openLoopRampRate(Constants.openLoopRate)
            .closedLoopRampRate(Constants.closedloopRate)
            .smartCurrentLimit(Constants.driveCurrentLimit);
        
        driveMotorConfigs.closedLoop
            .pidf(Constants.driveKP, Constants.driveKI, Constants.driveKD, Constants.driveKFF);
        
        driveMotorConfigs.encoder
            .positionConversionFactor(Constants.driveConversionPositionFactor)
            .velocityConversionFactor(Constants.driveConversionVelocityFactor);
            
        
        angleMotorConfigs
            .inverted(Constants.angleInvert)
            .idleMode(Constants.angleNeutralMode)
            .openLoopRampRate(Constants.openLoopRate)
            .closedLoopRampRate(Constants.closedloopRate)
            .smartCurrentLimit(Constants.angleCurrentLimit);

        angleMotorConfigs.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(Constants.angleKP, Constants.angleKI, Constants.angleKD, Constants.angleKFF);
          

        angleMotorConfigs.encoder
            .positionConversionFactor(Constants.angleConversionFactor);
        
        canCoderConfigs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1; //unsigned [0,1]
        canCoderConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    }
}
