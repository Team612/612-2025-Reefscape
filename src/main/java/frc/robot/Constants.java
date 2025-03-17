// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Constants {

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
        
        public static final int limIID = 0;


        public static final int bagID = 7; 
        public static double bagspeed = 0.60;

        public static final int bagCurrentLimit = 30;
        public static final boolean bagCurrentLimitEnable = true;

        public static double kP = 0.1;
        public static double kI = 0;
        public static double kD = 0;

        public static double pivotspeed = 0.50; 
        public static final double L1Position = 0.0;
        public static final double L2Position = 8.22; //26.126;
        public static final double L3Position = 8.22; //26.126;
        public static final double CoralStationPosition = 329.83; //347.451  341.212;
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

        public static final int magID = 3;
        public static final int magReached = 200;
        public static final int limEID = 9;


        public static final MAXMotionPositionMode positionMode = MAXMotionPositionMode.kMAXMotionTrapezoidal;
        public static final double maxElevatorSpeed = 0.3; //default units: revolutions per minutes (converted with the velocityConversionFactor)
        public static final double maxElevatorAcceleration = 1.0; //defualt units: RPM / s (converted with the velocityConversionFactor)
        public static final double marginOfError = 0.001; //in positions. Check .allowedClosedLoopError for more info

        public static final double kGearRatio = (5 / 1.0);  
        public static final double kSprocketPitchDiameter = Units.inchesToMeters(3.0);
        public static final double kPositionConversionFactor = (kSprocketPitchDiameter * Math.PI) / kGearRatio; //meters per revolutions
        public static final double kVelocityConversionFactor = kPositionConversionFactor / 60.0; // meters per rotations seconds

        public static double kP = 1.8;
        public static double kI = 0;
        public static double kD = 0;
        public static double kS = 0.2;
        public static double kG = 0.1;
        public static double kV = 0.0;

            
    public static final double basePosition = 0.0;
    public static final double L1Position = 0.035;
    public static final double L2Position = 0.245; //0.279
    public static final double L3Position = 0.654;
    public static final double CoralStationPosition = 0.318;//0.318
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
        public static final int candleID = 1;
    }

    public class DriverConstants {
        public static final int driverPort = 0;
        public static final int gunnerPort = 1;
        public static final int gunnerPort2 = 2;
        public static final double stickDeadband = 0.1;
    }

    public class AutoConstants {
        public static final String frontCamera = "FrontCamera";
        public static final String backCamera = "BackCamera";
        public static final double maxVelocity = 4.5;
        public static final double maxAcceleration = 1;
        public static final double maxAngularVelocity = Math.PI;
        public static final double maxAngularAcceleration = Math.PI/6;
        public static final Transform3d frontCameraTransform = new Transform3d(new Translation3d(Units.inchesToMeters(7.5),Units.inchesToMeters(0),Units.inchesToMeters(8)), new Rotation3d());
        public static final Transform3d backCameraTransform = new Transform3d(new Translation3d(Units.inchesToMeters(-15), Units.inchesToMeters(-12), Units.inchesToMeters(36)), new Rotation3d(0, Units.degreesToRadians(17), Units.degreesToRadians(180)));


        public static final double yApriltagDisplacement = 0.18; // 0.20
        public static final double xApriltagDisplacement = 0.25;
    }
  
    
    
    
}
