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
    public class PivotConstants {
        public static final int pivotID = 0;
        public static final boolean pivotInverted = true;
        public static final int pivotCurrentLimit = 30;
        public static final double boreOffset = 0;
        public static final IdleMode idleMode = IdleMode.kBrake;

        public static final int bagCurrentLimit = 30;
        public static final boolean bagCurrentLimitEnable = true;

        public static double pivotspeed = 0; 

    }

    public class IntakeConstants {
        public static final int bagID = 1; 
        public static double bagspeed = 0;
        

    }

    public class ElevatorConstants {
        public static final int elevatorID = 2;
        public static final boolean elevatorInverted = true;
        public static final int elevatorCurrentLimit = 30;
        public static final IdleMode idleMode = IdleMode.kBrake;
        public static double payloadspeed = 0;
    }
  
    
    
    public static double kPositionConversionFactor;
    public static final int encB = 0;
    public static final int encL1 = 10;
    public static final int encL2 = 20;
    
    

    public static final double kGearRatio = 5;  //idk
    public static final double kSprocketPitchDiameter = 1.504;
    // public static final int neoPivotID = 0;
}
