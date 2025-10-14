package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class LeaveZone extends Command {
  private final Swerve m_drivetrain;
  private final double tolerance = 0.1; // Tolerance for reaching the target position (in meters)
double timer;

  public LeaveZone(Swerve drivetrain) {
    m_drivetrain = drivetrain;
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = 0;
    // Reset odometry or encoders if necessary
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    timer += 1;
    // Use proportional control to calculate speeds
    double xSpeed = 0.8;// Adjust the constant as needed
    double ySpeed = 0.0;

    m_drivetrain.resetGyro();

    // Create ChassisSpeeds object for field-relative movement
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, 0.0, m_drivetrain.getHeading());

    // Command the drivetrain to move
    m_drivetrain.setRobotBassedOffFieldChassisSpeeds(speeds);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer > 150; // Check if within tolerance of 1000 scheduler runs
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the robot
    m_drivetrain.setRobotBassedOffFieldChassisSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0));
  }
}