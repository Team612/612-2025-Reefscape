// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
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

        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;

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

        public static final MAXMotionPositionMode positionMode = MAXMotionPositionMode.kMAXMotionTrapezoidal;
        public static final double maxElevatorSpeed = 4.0; //default units: revolutions per minutes (converted with the velocityConversionFactor)
        public static final double maxElevatorAcceleration = 1.0; //defualt units: RPM / s (converted with the velocityConversionFactor)
        public static final double marginOfError = 0; //in positions. Check .allowedClosedLoopError for more info

        public static final double kGearRatio = (5 / 1.0);  
        public static final double kSprocketPitchDiameter = 1.504;
        public static final double kPositionConversionFactor = (kSprocketPitchDiameter * Math.PI) / kGearRatio; //meters per rotations
        public static final double kVelocityConversionFactor = kPositionConversionFactor / 60.0; // meters per rotations seconds

        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;

            
    public static final int basePosition = 0;
    public static final int L1Position = 10;
    public static final int L2Position = 20;

    }

    public class ClimbConstants {
        public static final int servoID = 0;
        public static final int pivotID = 0;
        public static final int climbCurrentLimit = 30;
        public static final boolean inverted = false;
        public static final IdleMode idleMode = IdleMode.kBrake;

        public static final double kGearRatio = (245 / 1.0);
        public static final double kSprocketPitchDiameter = Units.inchesToMeters(7.38);

        public static final double kPositionConversionFactor = (kSprocketPitchDiameter * Math.PI) / kGearRatio;
        public static final double kVelocityConversionFactor = kPositionConversionFactor / 60.0;

        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;


        public static final int servoOpenPosition = 1;
        public static final int servoClosePosition = 0;

        

    }
  
    
    

    
}
