// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Constants {
    public static final int elevatorID = 2;
    // public static final int elevatorID2 = 2;
    public static double payspeed = 0;
    public static double kPositionConversionFactor;

    public static final int encB = 0;
    public static final int encL1 = 10;
    public static final int encL2 = 20;
    
    public static final double boreOffset = 0;
    // public static final int neoPivotID = 0;
}
