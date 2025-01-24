// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import java.io.IOException;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.LimelightHelpers.RawFiducial;

public class PoseEstimator extends SubsystemBase {

  Vision visionSubsystem;
  Mecanum mecanum;

  Pose3d estimatedPose;

  AprilTagFieldLayout layout;
  MecanumDrivePoseEstimator drivePoseEstimator;

  // StructPublisher<Pose2d> publisher;
  // StructArrayPublisher<Pose2d> arrayPublisher;
  // {
 
  // }

  public static final double FIELD_LENGTH_METERS = Units.inchesToMeters(651.25);
  public static final double FIELD_WIDTH_METERS = Units.inchesToMeters(323.25);
  private double previousPipelineTimestamp = 0;

  static PoseEstimator estimator = null;
  
  public PoseEstimator() {
    mecanum = Mecanum.getInstance();
    visionSubsystem = Vision.getInstance();
       try{
          layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        }
        catch(IOException e) {
                DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
                layout = null;
        }
    drivePoseEstimator = new MecanumDrivePoseEstimator(
      Constants.DrivetrainConstants.kDriveKinematics, 
      mecanum.getPigeonAngle(), 
      mecanum.getMecanumDriveWheelPositions(), 
      new Pose2d()
    );
}

  public static PoseEstimator getPoseEstimatorInstance() {
    if (estimator == null) {
      estimator = new PoseEstimator();
    }
    return estimator;
  }


  public void updatePose(MecanumDrivePoseEstimator poseEstimator){ 
      LimelightResults results = LimelightHelpers.getLatestResults(Constants.limeName);
      LimelightTarget_Fiducial[] fiducials = results.targets_Fiducials;

      for (LimelightTarget_Fiducial fiducial : fiducials) {
          Pose3d robotPoseField;
          Pose3d tagPoseField = layout.getTagPose((int) fiducial.fiducialID).orElse(new Pose3d());

          Pose3d tagPoseRobot = fiducial.getTargetPose_RobotSpace();

          Translation3d invertedTranslation = tagPoseRobot.getTranslation().unaryMinus();
          Rotation3d invertedRotation = tagPoseRobot.getRotation().unaryMinus();
          Pose3d tagPoseRobotInverse = new Pose3d(invertedTranslation, invertedRotation);
          Translation3d fieldTranslation = tagPoseField.getTranslation().plus(tagPoseRobotInverse.getTranslation());
          Rotation3d fieldRotation = tagPoseField.getRotation().plus(tagPoseRobotInverse.getRotation());

          robotPoseField = new Pose3d(fieldTranslation, fieldRotation);
          Pose3d estimatedPose = robotPoseField;
          if (estimatedPose.getX() >= 0.0 && estimatedPose.getX() <= FIELD_LENGTH_METERS  && estimatedPose.getY() >= 0.0 && estimatedPose.getY() <= FIELD_WIDTH_METERS) {
          }
          else{
            estimatedPose = new Pose3d();
          }
      }
  }


  @Override
  public void periodic() {
    drivePoseEstimator.update(mecanum.getPigeonAngle(), mecanum.getMecanumDriveWheelPositions());
    updatePose(drivePoseEstimator);
    SmartDashboard.putNumber("X", getPose().getX());
    SmartDashboard.putNumber("Y", getPose().getY());
    SmartDashboard.putNumber("Z", getPose().getZ());
  }


  public Pose3d getPose() {
    return estimatedPose;
  }

  public void setPose(Pose3d newPose) {
      estimatedPose = newPose;
  }
}