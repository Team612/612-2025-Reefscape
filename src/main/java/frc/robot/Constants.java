// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.util.SwerveModuleConstants;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

import com.ctre.phoenix6.swerve.SwerveModule;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

/** Add your docs here. */
public class Constants {
    public class SwerveConstants{
        //swerve constants
        public static final double trackWidth = Units.inchesToMeters(27);
        public static final double wheelBase = Units.inchesToMeters(27);
        public static final double wheelDiameter = Units.inchesToMeters(3.75);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1
        public static final double angleGearRatio = (150.0 / 7.0); // 12.8:1

        public static final SwerveDriveKinematics swerveKinematics =
            new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));


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
        public static final int driveCurrentLimit = 30;
        public static final int angleCurrentLimit = 30;


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

         /* driver gunner ports */
        public static final int driverPort = 0;
        public static final int gunnerPort = 1;
        public static final double stickDeadband = 0.1;

        /*pigeon angle */
        public static final int pigeonID = 0;

        public static final class Mod0 {
        public static final int driveMotorID = 4;
        public static final int angleMotorID = 5;
        public static final int canCoderID = 0;
        public static final Rotation2d desiredAngle = Rotation2d.fromDegrees(234.66 - 180 + 1); 
        public static final SwerveModuleConstants constants =
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, desiredAngle);
        }

        public static final class Mod1 {
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 3;
            public static final int canCoderID = 1;
            public static final Rotation2d desiredAngle = Rotation2d.fromDegrees(8.70 + 180); 
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, desiredAngle);
        }
        
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 3;
            public static final Rotation2d desiredAngle = Rotation2d.fromDegrees(99.17 - 3);
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, desiredAngle);
        }
        
        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 8;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 2;
            public static final Rotation2d desiredAngle = Rotation2d.fromDegrees(-1.58 + 180 + 1);
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, desiredAngle);
        }
        


    }

    //below was used for mecanum
    public class DrivetrainConstants{

        public static final int pigeonID = 0;

        public static final int SPARK_FL = 2;
        public static final int SPARK_BR = 4;
        public static final int SPARK_BL = 3;
        public static final int SPARK_FR = 1;

        public static final double kP = 0.01;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        

        // Need to get locations of wheels relative to center of robot
        public static final Translation2d m_frontLeftLocation = new Translation2d(0.267, 0.305);
        public static final Translation2d m_frontRightLocation = new Translation2d(0.267, -0.305);
        public static final Translation2d m_backLeftLocation = new Translation2d(-0.267, 0.305);
        public static final Translation2d m_backRightLocation = new Translation2d(-0.267, -0.305);

        public static final MecanumDriveKinematics m_kinematics = new MecanumDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
        );

  
        public static final double kWheelBase =  Units.inchesToMeters(21); // width from center of back to front wheels (center of the wheel)
        public static final double kTrackWidth = Units.inchesToMeters(24); // width from right to left wheels (center of the wheel)
        public static final double kGearReduction = 16;
        public static final double kS = 0.15950; 
        public static final double kV = 4.6647;  
        public static final double kA = 0;
        public static final int currentLimit = 30;

        public static final SimpleMotorFeedforward kFeedforward =
         new SimpleMotorFeedforward(Constants.DrivetrainConstants.kS, Constants.DrivetrainConstants.kV, Constants.DrivetrainConstants.kA);
     
        public static final double kWheelDiameterMeters = 0.1524; 
        public static final double kPositionConversionFactor =
            ((kWheelDiameterMeters * Math.PI)) / (kGearReduction);

        public static final double kVelocityConversionFactor =
            kPositionConversionFactor / 60.0;

    }

    public class IntakeConstants {
        public static final int pivotID = 6;
        public static final boolean pivotInverted = false;
        public static final int pivotCurrentLimit = 30;
        public static final double boreOffset = 0;
        public static final IdleMode idleMode = IdleMode.kBrake;
        public static final boolean bagInverted = false;
        public static final double kGearRatio = (125.0 / 1.0);
        public static final double kSprocketPitchDiameter = Units.inchesToMeters(2.87);
        public static final double kPositionConversionFactor = 360  / kGearRatio;
            // ((kSprocketPitchDiameter * Math.PI)) / (kGearRatio);
        public static final double kAngularPositionConversionFactor = 
            360.0 / kPositionConversionFactor;
        public static final double kVelocityConversionFactor =
            kPositionConversionFactor / 60.0;
        

        public static final int bagID = 7; 
        public static double bagspeed = 0.60;

        public static final int bagCurrentLimit = 30;
        public static final boolean bagCurrentLimitEnable = true;

        public static double kP = 0.1;
        public static double kI = 0;
        public static double kD = 0;

        public static double pivotspeed = 0.30; 
        public static final double L1Position = 0.0;
        public static final double L2Position = 26.126;
        public static final double L3Position = 26.126;
        public static final double CoralStationPosition = 347.451;
        public static final double maxPivotL1Angle = 70;
        public static final double maxVelocity = 0.3;
        public static final double maxAcceleration = 0.3;
        public static final double pivotThreshold = 0.5;
        public static final double intakeThreshold = 2;//in degrees
        public static final boolean pivotWrapping = true;

    }

    public class ElevatorConstants {
        public static final int elevatorID = 5;
        public static final boolean elevatorInverted = true;
        public static final int elevatorCurrentLimit = 30;
        public static final IdleMode idleMode = IdleMode.kBrake;
        public static final double elePos = 0;
        public static double payloadUpSpeed = 0.12;
        public static double payloadDownSpeed = 0.15;
       

        public static final MAXMotionPositionMode positionMode = MAXMotionPositionMode.kMAXMotionTrapezoidal;
        public static final double maxElevatorSpeed = 0.3; //default units: revolutions per minutes (converted with the velocityConversionFactor)
        public static final double maxElevatorAcceleration = 1.0; //defualt units: RPM / s (converted with the velocityConversionFactor)
        public static final double marginOfError = 0.001; //in positions. Check .allowedClosedLoopError for more info

        public static final double kGearRatio = (5 / 1.0);  
        public static final double kSprocketPitchDiameter = Units.inchesToMeters(3.0);
        public static final double kPositionConversionFactor = (kSprocketPitchDiameter * Math.PI) / kGearRatio; //meters per revolutions
        public static final double kVelocityConversionFactor = kPositionConversionFactor / 60.0; // meters per rotations seconds

        public static double kP = 1.6;
        public static double kI = 0;
        public static double kD = 0;
        public static double kS = 0.2;
        public static double kG = 0.1;
        public static double kV = 0.0;

            
    public static final double basePosition = 0.0;
    public static final double L1Position = 0.035;
    public static final double L2Position = 0.236; //0.279
    public static final double L3Position = 0.654;
    public static final double CoralStationPosition = 0.316;
    // public static final double L1Minimum = 0.3 //the minimum position that allows for maximum movement for the intake pivot
    public static final double elevatorThreshold = 0.01;


    }

    public class ClimbConstants {
        public static final int servoID = 0;
        public static final int pivotID = 8;
        public static final int climbCurrentLimit = 30;
        public static final boolean inverted = true;
        public static final IdleMode idleMode = IdleMode.kBrake;

        public static final double kGearRatio = (245 / 1.0);
        public static final double kSprocketPitchDiameter = Units.inchesToMeters(7.38);

        public static final double kPositionConversionFactor = (kSprocketPitchDiameter * Math.PI) / kGearRatio; // meters per revolution
        public static final double kAngularPositionConversionFactor = 360.0 / kGearRatio;
        public static final double kVelocityConversionFactor = kPositionConversionFactor / 60.0;

        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;


        public static final double servoOpenPosition = 0; // 1
        public static final double servoClosePosition = 1; // 0

        public static final double pivotOutAngle = 0;
        public static final double pivotInAngle = 0;
        public static double pivotSpeed = 0.8;
        

    }

    public class LedConstants {
        public static final int candleID = 0;
    }

    public class DriverConstants {
        public static final int driverPort = 0;
        public static final int gunnerPort = 1;
        public static final double stickDeadband = 0.1;
    }

    public class AutoConstants {
        public static final String frontCamera = "FrontCamera";
        public static final String backCamera = "BackCamera";
        public static final double maxVelocity = 4.5;
        public static final double maxAcceleration = 1;
        public static final double maxAngularVelocity = Math.PI;
        public static final double maxAngularAcceleration = Math.PI/6;

        public static final double yApriltagDisplacement = 1;
        public static final double xApriltagDisplacement = 0.5;
    }
  
    
    
    
}
