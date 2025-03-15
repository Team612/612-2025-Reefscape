package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class Constants {
    public static final int XboxPortNumber = 0;

    // gyro port
    public static final int gyroID = 0;
  
    // sets the minimum controller request percent
    public static final double Deadband = 0.05;
  
    // sets the proportional constant for the angle motor PID
    public static final double kp = 0.5;
  
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