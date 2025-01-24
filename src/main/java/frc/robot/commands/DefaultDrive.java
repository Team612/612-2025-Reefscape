package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Mecanum;
import java.util.function.DoubleSupplier;

public class DefaultDrive extends Command {
  private Mecanum Mecanum;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

  public DefaultDrive(
      Mecanum drive,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup) {
    this.Mecanum = drive;
    addRequirements(drive);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
  }

  @Override
  public void execute() {

    double translationVal =
        translationLimiter.calculate(
            MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband));
    double strafeVal =
        strafeLimiter.calculate(
            MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband));
    double rotationVal =
        rotationLimiter.calculate(
            MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband));

    MecanumDriveKinematics m_kinematics = new MecanumDriveKinematics(
      Constants.DrivetrainConstants.m_frontLeftLocation, Constants.DrivetrainConstants.m_frontRightLocation, Constants.DrivetrainConstants.m_backLeftLocation, Constants.DrivetrainConstants.m_backRightLocation
    );

    ChassisSpeeds speeds = new ChassisSpeeds(translationVal, strafeVal, rotationVal);
    // Convert to wheel speeds
    MecanumDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(speeds);
    // Get the individual wheel speeds
    double frontLeft = wheelSpeeds.frontLeftMetersPerSecond;
    double frontRight = wheelSpeeds.frontRightMetersPerSecond;
    double backLeft = wheelSpeeds.rearLeftMetersPerSecond;
    double backRight = wheelSpeeds.rearRightMetersPerSecond;


    /* Drive */
    Mecanum.driveMecanum(frontLeft, backLeft, frontRight, backRight);
  }
}