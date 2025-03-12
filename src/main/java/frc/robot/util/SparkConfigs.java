// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;

/** Add your docs here. */
public class SparkConfigs {
    public SparkMaxConfig driveMotorConfigs;
    public SparkMaxConfig angleMotorConfigs;

    public SparkConfigs(){
        driveMotorConfigs = new SparkMaxConfig();
        angleMotorConfigs = new SparkMaxConfig();
   
        
        driveMotorConfigs
            .inverted(Constants.SwerveConstants.driveInvert)
            .idleMode(Constants.SwerveConstants.driveNeutralMode)
            .openLoopRampRate(Constants.SwerveConstants.openLoopRate)
            .closedLoopRampRate(Constants.SwerveConstants.closedloopRate)
            .smartCurrentLimit(Constants.SwerveConstants.driveCurrentLimit);
        
        driveMotorConfigs.closedLoop
            .pidf(Constants.SwerveConstants.driveKP, Constants.SwerveConstants.driveKI, Constants.SwerveConstants.driveKD, Constants.SwerveConstants.driveKFF);
        
        driveMotorConfigs.encoder
            .positionConversionFactor(Constants.SwerveConstants.driveConversionPositionFactor)
            .velocityConversionFactor(Constants.SwerveConstants.driveConversionVelocityFactor);
            
        
        angleMotorConfigs
            .inverted(Constants.SwerveConstants.angleInvert)
            .idleMode(Constants.SwerveConstants.angleNeutralMode)
            .openLoopRampRate(Constants.SwerveConstants.openLoopRate)
            .closedLoopRampRate(Constants.SwerveConstants.closedloopRate)
            .smartCurrentLimit(Constants.SwerveConstants.angleCurrentLimit);

        angleMotorConfigs.closedLoop
            .pidf(Constants.SwerveConstants.angleKP, Constants.SwerveConstants.angleKI, Constants.SwerveConstants.angleKD, Constants.SwerveConstants.angleKFF);

        angleMotorConfigs.encoder
            .positionConversionFactor(Constants.SwerveConstants.angleConversionFactor);
    }
}