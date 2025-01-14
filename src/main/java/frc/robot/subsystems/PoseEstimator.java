// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
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

public class PoseEstimator extends SubsystemBase {
  SwerveDrivePoseEstimator drivePoseEstimator;
  Vision visionSubsystem;
  Swerve swerve;
  private Field2d fieldLayout;
  Pose2d pose = new Pose2d();
  Pose2d estimatedPose;
  
  StructPublisher<Pose2d> publisher;
  StructArrayPublisher<Pose2d> arrayPublisher;
  
  public static final double FIELD_LENGTH_METERS = Units.inchesToMeters(651.25);
  public static final double FIELD_WIDTH_METERS = Units.inchesToMeters(323.25);
  private double previousPipelineTimestamp = 0;

  static PoseEstimator estimator = null;
  
  public PoseEstimator() {
    swerve = Swerve.getInstance();
    visionSubsystem = Vision.getInstance();
    fieldLayout = new Field2d();
    SmartDashboard.putData("Field", fieldLayout);

    drivePoseEstimator = new SwerveDrivePoseEstimator(
      Constants.swerveKinematics, 
      swerve.getPigeonAngle(), 
      swerve.getSwervePoses(), 
      new Pose2d()
    );
    
    pose = visionSubsystem.getAprilTagPose();
  }

  public static PoseEstimator getPoseEstimatorInstance() {
    if (estimator == null) {
      estimator = new PoseEstimator();
    }
    return estimator;
  }


  public void updatePose(PoseEstimator poseEstimator, int camID){    
    if(LimelightHelpers.getLatestResults(Constants.limeName).targets_Fiducials.length > 0) {
      estimatedPose = visionSubsystem.getAprilTagPose();

        if (estimatedPose.getX() >= 0.0 && estimatedPose.getX() <= FIELD_LENGTH_METERS  && estimatedPose.getY() >= 0.0 && estimatedPose.getY() <= FIELD_WIDTH_METERS) {
            
          for (LimelightHelpers.LimelightTarget_Fiducial target : visionSubsystem.getAprilTags()) {
              Pose2d targetPose = visionSubsystem.returnTagPose(target);
              // Transform3d bestTarget = target.getBestCameraToTarget();
              // if (target.getPoseAmbiguity() <= .2) {
              //   previousPipelineTimestamp = estimatedRobotPose.timestampSeconds;
              // }
            }
        }
      }
    }


  @Override
  public void periodic() {
    drivePoseEstimator.update(swerve.getPigeonAngle(), swerve.getSwervePoses());
    publisher.set(drivePoseEstimator.getEstimatedPosition());
    fieldLayout.setRobotPose(getPose());
  }


  public Pose2d getPose() {
    return drivePoseEstimator.getEstimatedPosition();
  }

  public void setPose(Pose2d newPose) {
    drivePoseEstimator.resetPosition(swerve.getPigeonAngle(), swerve.getSwervePoses(), newPose);
  }
}