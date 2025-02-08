// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Constants {
    public static final double maxSpeed = 6.0; // meters per second
    public static final double maxAngularVelocity = 3 * Math.PI;
    public static final double maxAcceleration = 4.5;
    public static final double maxAngularAcceleration = Math.PI/6;
    public static final int pigeonID = 0;
 //Distance btwn centers of right and left wheels

    public class DrivetrainConstants{
        // Starting from here we don't know the stuff
        // Get IDs from rev client

        public static final int SPARK_FL = 2;
        public static final int SPARK_BR = 4;
        public static final int SPARK_BL = 3;
        public static final int SPARK_FR = 1;

        public static final double kP = 0.01;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        

        // Need to get locations of wheels relative to center of robot
        public static final Translation2d m_frontLeftLocation = new Translation2d(0.305, 0.305);
        public static final Translation2d m_frontRightLocation = new Translation2d(0.305, -0.305);
        public static final Translation2d m_backLeftLocation = new Translation2d(-0.305, 0.305);
        public static final Translation2d m_backRightLocation = new Translation2d(-0.305, -0.305);

        public static final MecanumDriveKinematics m_kinematics = new MecanumDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
        );

        // We know none of this
        public static final double kWheelBase =  Units.inchesToMeters(21.5); // width from center of back to front wheels (center of the wheel)
        public static final double kTrackWidth = Units.inchesToMeters(24); // width from right to left wheels (center of the wheel)
        public static final double kGearReduction = 16;
        public static final double kS = 0.15950; 
        public static final double kV = 4.6647;  
        public static final double kA = 0;

        public static final SimpleMotorFeedforward kFeedforward =
         new SimpleMotorFeedforward(Constants.DrivetrainConstants.kS, Constants.DrivetrainConstants.kV, Constants.DrivetrainConstants.kA);
     
        // this stuff we know
        public static final double kWheelDiameterMeters = 0.1524; 
        public static final double kEncoderDistancePerPulse =
            ((kWheelDiameterMeters * Math.PI)) / (kGearReduction);
        public static final MecanumDriveKinematics kDriveKinematics =
            new MecanumDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
        );
    }

    /* driver gunner ports */
    public static final int driverPort = 0;
    public static final int gunnerPort = 1;
    public static final double stickDeadband = 0.1;
}
