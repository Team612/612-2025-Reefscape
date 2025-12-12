package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class Constants {
    public static final double abosulteMaxCurrentLimit = 100; // keep in mind in practice the highest it will realistically go it 80% of this
    public static final double steerSupplyCurrent = 5;
    public static final double driveSupplyCurrent = (abosulteMaxCurrentLimit-steerSupplyCurrent*4)/4;

    public static final String frontCameraName = "FrontCamera";

    public static final double trackWidth = 0.550;
    public static final double wheelBase = 0.555;

    public static final double wheelDiameter = 0.096;
    // public static final double gearRatio = 5.91;
    public static final double gearRatio = 6;
    // public static final double gearRatio = 6.75;
    public static final double rotationsToMeters = ((Math.PI * wheelDiameter) / gearRatio)*1.3;

    public static final double metersPerSecondToPercent = 0.2;
    public static final double radiansPerSecondToPercent = 0.0127835295656;

    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    public static final double xPercent = 1;
    public static final double yPercent = 1;
    public static final double zPercent = 1;

    public static final double zNecessaryOffset = (zPercent)/(Math.sqrt((trackWidth/2)*(trackWidth/2)+(wheelBase/2)*(wheelBase/2)));

    public static final int controllerPortNumber = 0;

    public static final int gyroID = 0;

    public static final int CANdleID = 1;

    public static final double DEADBAND = 0.05;

    public static final double kp = 0.5;

    public static final double frontLEncoderOffset = -0.01;
    public static final int frontLSteerMotorID = 7;
    public static final int frontLDriveMotorID = 6;
    public static final int frontLCANcoderID = 4;

    public static final double frontREncoderOffset = -0.473;
    public static final int frontRSteerMotorID = 5;
    public static final int frontRDriveMotorID = 4;
    public static final int frontRCANcoderID = 3;

    public static final double backLEncoderOffset = 0.272;
    public static final int backLSteerMotorID = 1;
    public static final int backLDriveMotorID = 8;
    public static final int backLCANcoderID = 1;

    public static final double backREncoderOffset = -0.352;
    public static final int backRSteerMotorID = 3;
    public static final int backRDriveMotorID = 2;
    public static final int backRCANcoderID = 2;
}
