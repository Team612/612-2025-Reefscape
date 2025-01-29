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
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Constants {
    public static final double trackWidth = Units.inchesToMeters(27);
    public static final double wheelBase = Units.inchesToMeters(27);
    public static final double wheelDiameter = Units.inchesToMeters(3.75);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1
    public static final double angleGearRatio = (150.0 / 7.0); // 12.8:1
    public static final String limeName = "limelight";

    /* Swerve Profiling Values */
    public static final double maxSpeed = 4.5; // meters per second
    public static final double maxAngularVelocity = Math.PI;
    public static final double maxAcceleration = 1;
    public static final double maxAngularAcceleration = Math.PI/6;

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    /* Motor Inverts */
    public static final boolean driveInvert = false;
    public static final boolean angleInvert = true;

    /* Open and Closed Ramp Rates */
    public static final double openLoopRate = 0.25;
    public static final double closedloopRate = 0;

    /* Current Limits */
    public static final int driveCurrentLimit = 60;
    public static final int angleCurrentLimit = 60;


    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;

    /* Angle Motor PID Values */
    public static final double angleKP = 0.01;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.0;
    public static final double angleKFF = 0.0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.1;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKFF = 0.0;

    /* Drive Motor Characterization Values */
    public static final double driveKS = 0.58453; //0.22005
    public static final double driveKV = 2.40269; //2.74490
    public static final double driveKA = 0;

    //have to tune manually
    public static final double kPXController = 5; // ~ 1cm error
    public static final double kPYController = 1;
    public static final double kPThetaController = 4.5; 

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor =
        (wheelDiameter * Math.PI) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;


    /* navx angle offset */
    public static final double navxAngleOffset = 0;
    public static final int pigeonID = 2;
 //Distance btwn centers of right and left wheels

    public class DrivetrainConstants{
        // Starting from here we don't know the stuff
        // Get IDs from rev client

        public static final int SPARK_FL = 2;
        public static final int SPARK_BR = 4;
        public static final int SPARK_BL = 3;
        public static final int SPARK_FR = 1;

        // Need to get locations of wheels relative to center of robot
        public static final Translation2d m_frontLeftLocation = new Translation2d(-0.305, 0.305);
        public static final Translation2d m_frontRightLocation = new Translation2d(0.305, 0.305);
        public static final Translation2d m_backLeftLocation = new Translation2d(-0.305, -0.305);
        public static final Translation2d m_backRightLocation = new Translation2d(0.305, -0.305);
        // We know none of this
        public static final double kWheelBase =  Units.inchesToMeters(21.5); // width from center of back to front wheels (center of the wheel)
        public static final double kTrackWidth = Units.inchesToMeters(24); // width from right to left wheels (center of the wheel)
        public static final double kGearReduction = 16;

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
