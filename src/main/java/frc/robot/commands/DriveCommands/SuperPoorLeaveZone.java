package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SuperPoorLeaveZone extends Command {
  private final Swerve m_drivetrain;
  private final Vision m_vision;
  private final Pose2d targetPose;
  private final double tolerance = 0.1; // Tolerance for reaching the target position (in meters)

  public SuperPoorLeaveZone(Swerve drivetrain, Vision vision, Pose2d targetPose) {
    m_drivetrain = drivetrain;
    m_vision = vision;
    this.targetPose = targetPose;
    addRequirements(m_drivetrain, m_vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Reset odometry or encoders if necessary
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = m_drivetrain.getPose();
    Translation2d currentPosition = currentPose.getTranslation();
    Translation2d targetPosition = targetPose.getTranslation();

    double xError = targetPosition.getX() - currentPosition.getX();
    double yError = targetPosition.getY() - currentPosition.getY();

    // Use proportional control to calculate speeds
    double xSpeed = xError * 0.5; // Adjust the constant as needed
    double ySpeed = yError * 0.5;

    // Limit speeds to maximum allowed values
    xSpeed = Math.min(Math.max(xSpeed, -Constants.DrivetrainConstants.MAX_SPEED), Constants.DrivetrainConstants.MAX_SPEED); // Clamp speed between -0.5 and 0.5
    ySpeed = Math.min(Math.max(ySpeed, -Constants.DrivetrainConstants.MAX_SPEED), Constants.DrivetrainConstants.MAX_SPEED);

    // Create ChassisSpeeds object for field-relative movement
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, 0.0, m_drivetrain.getHeading());

    // Command the drivetrain to move
    m_drivetrain.setRobotBassedOffFieldChassisSpeeds(speeds);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Pose2d currentPose = m_drivetrain.getPose();
    Translation2d currentPosition = currentPose.getTranslation();
    Translation2d targetPosition = targetPose.getTranslation();

    double distanceToTarget = currentPosition.getDistance(targetPosition);
    return distanceToTarget < tolerance; // Check if within tolerance
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the robot
    m_drivetrain.setRobotBassedOffFieldChassisSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0));
  }
}