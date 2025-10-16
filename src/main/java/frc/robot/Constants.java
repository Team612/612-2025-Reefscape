package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kGunnerControllerPort = 1;

    public static final int kGunnerDriverStationPort1 = 1;
    public static final int kGunnerDriverStationPort2 = 2;
  }

  public static class DriveConstants {
    public static final int gyroID = 0;

    // swerve module 0 constants, front left
    // when the absolute encoder reads the 0.63 it is actually at 0
    public static final double frontLEncoderOffset = 0.63;
    public static final int frontLSteerMotorID = 7;
    public static final int frontLDriveMotorID = 6;
    public static final int frontLCANcoderID = 0;

    // swerve module 1 constants, front right
    // when the absolute encoder reads 0.02 it is actually at 0
    public static final double frontREncoderOffset = 0.735;
    public static final int frontRSteerMotorID = 5;
    public static final int frontRDriveMotorID = 4;
    public static final int frontRCANcoderID = 2;

    // swerve module 2 constants, back left
    // when the absolute encoder reads 0.735 it is actually at 0
    public static final double backLEncoderOffset = 0.459;
    public static final int backLSteerMotorID = 11;
    public static final int backLDriveMotorID = 8;
    public static final int backLCANcoderID = 3;

    // swerve module 3 constants, back right
    // when the absolute encoder reads 0.994 it is actually at 0
    public static final double backREncoderOffset = 0.2;
    public static final int backRSteerMotorID = 3;
    public static final int backRDriveMotorID = 2;
    public static final int backRCANcoderID = 1;

    // measured values
    public static final double maxMetersPerSecondSpeed = 4.29184549356;
    public static final double trackWidth = 0.605;
    public static final double wheelBase = 0.605;
    public static final double tickToMetersConstant = 1.04; // !! has not been measured irl yet !!

    // desired values
    public static final double xPercent = 1;
    public static final double yPercent = 1;
    public static final double zPercent = 0.3;
    public static final double kp = 0.5;
    public static final double DEADBAND = 0.05;
    public static final double simpleLeaveZoneSpeed = 0.2;
    public static final int simpleLeaveZoneTime = 100;

    // derived values, hope you like math
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    public static final double radiusInMeters = Math.sqrt((trackWidth/2)*(trackWidth/2)+(wheelBase/2)*(wheelBase/2));
    public static final double zNecessaryOffset = zPercent/radiusInMeters;

    public static final double metersPerSecondToPercent = 1/maxMetersPerSecondSpeed;

    public static final double maxRadiansPerSecondSpeed = maxMetersPerSecondSpeed/radiusInMeters;
    public static final double radiansPerSecondToPercent = 1/maxRadiansPerSecondSpeed;
  }

  public static class PayloadConstants {
    public static final int elevatorID = 15;
    public static final int pivotID = 16;
    public static final double initializeDEADBAND = 0.07;
    public static final double controllingDEADBAND = 0.05;

    public static final double elevatorkp = 1;
    public static final double L2Position = -0.271;
    public static final double L3Position = -0.683;
    public static final double CoralStationElevatorPosition = -0.301235;
    public static final double bottomAlgaePosition = -0.271;
    public static final double topAlgaePosition = -0.663;
    public static final double topElevatorPosition = -0.76035;
    public static final double elevatorZeroSpeed = 0.15;
    public static final double maxSafeHeight = -0.28;
    public static final double startingElevatorPos = CoralStationElevatorPosition;
    public static final double driveStationElevatorUpSpeed = 0.12;
    public static final double driveStationElevatorDownSpeed = 0.15;
    public static final double maxGunnerElevatorControlSpeed = 0.3;

    public static final double minPivotSpeed = 0.65; // dont move slow as hell if we are close
    public static final double intakekp = 0.015; // move faster if you are further away we dont have all day
    public static final double coralScoringPosition = -21;
    public static final double algaeIntakePosition = -66.71;
    public static final double minSafePivotPosition = -70;
    public static final double CoralStationIntakePosition = -325.512;
    public static final double backIntakePosition = -338.721;
    public static final double startingPivotPos = CoralStationIntakePosition;
    public static final double driveStationPivotSpeed = 0.65;
    public static final double maxGunnerPivotControlSpeed = 0.7;
  }

  public static class BagConstants {
    public static final int bagID = 17;
    public static final double autoBagSpeed = 0.9;
    public static final int autoBagTime = 50;
    public static final double driverBagSpeed = 0.6;
  }
}
