package frc.robot;

import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static final int XboxPortNumber = 0;

    public class SwerveConstants {
        // gyro port
        public static final int gyroID = 0;
    
        // sets the minimum controller request percent
        public static final double Deadband = 0.05;
    
        // sets the proportional constant for the angle motor PID
        public static final double drivekP = 0.5;
        public static final double drivekI = 0.0;
        public static final double drivekD = 0.0;

        public static final double anglekP = 0.5;
        public static final double anglekI = 0.0;
        public static final double anglekD = 0.0;
        public static final int currentLimit = 25;
    
        // this is a very important constant it measures how much motor percent it takes to travel 1 m/s
        public static final double metersPerSecondtoMotorPercentConstant = 0.233;
    
        // used to desaturate the wheel speeds if we request them to go over this limit
        public static final double MAX_SPEED = 1/metersPerSecondtoMotorPercentConstant; // m/s
    
        // this controls our desired m/s inputs from the controller
        public static final double xMultiple = MAX_SPEED;
        public static final double yMultiple = MAX_SPEED;
        // this controls our desired rad/s inputs from the controller
        public static final double zMultiple = 3;
    
        // used to instantiate swerve kinematics
        public static final double trackWidth = 0.605;
        public static final double wheelBase = 0.605;
    
        // swerve module 0 constants, front left
        // when the absolute encoder reads the 0.63 it is actually at 0
        public static final double mod0EncoderOffset = 0.63;
        public static final int mod0AngleMotorID = 7;
        public static final int mod0DriveMotorID = 6;
        public static final int mod0CANcoderID = 0;
    
        // swerve module 1 constants, front right
        // when the absolute encoder reads 0.02 it is actually at 0
        public static final double mod1EncoderOffset = 0.735;
        public static final int mod1AngleMotorID = 5;
        public static final int mod1DriveMotorID = 4;
        public static final int mod1CANcoderID = 2;
    
        // swerve module 2 constants, back left
        // when the absolute encoder reads 0.735 it is actually at 0
        public static final double mod2EncoderOffset = 0.459;
        public static final int mod2AngleMotorID = 1;
        public static final int mod2DriveMotorID = 8;
        public static final int mod2CANcoderID = 3;
    
        // swerve module 3 constants, back right
        // when the absolute encoder reads 0.994 it is actually at 0
        public static final double mod3EncoderOffset = 0.2;
        public static final int mod3AngleMotorID = 3;
        public static final int mod3DriveMotorID = 2;
        public static final int mod3CANcoderID = 1;

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));
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

        public static double pivotspeed = 0.50; 
        public static final double L1Position = 0.0;
        public static final double L2Position = 26.126;
        public static final double L3Position = 26.126;
        public static final double CoralStationPosition = 341.212; //347.451  341.212;
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
    public static final double CoralStationPosition = 0.284;//0.318
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


        public static final double yApriltagDisplacement = 0.20;
        public static final double xApriltagDisplacement = 0.25;
    }
}