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
import org.photonvision.*;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;

public class PoseEstimator extends SubsystemBase {

  Vision visionSubsystem;
  Swerve swerve;

  Pose3d estimatedPose;

  AprilTagFieldLayout layout;
  SwerveDrivePoseEstimator drivePoseEstimator;
  PhotonPoseEstimator photonEstimator;

  // StructPublisher<Pose2d> publisher;
  // StructArrayPublisher<Pose2d> arrayPublisher;
  // {
 
  // }

  public static final double FIELD_LENGTH_METERS = Units.inchesToMeters(651.25);
  public static final double FIELD_WIDTH_METERS = Units.inchesToMeters(323.25);
  private double previousPipelineTimestamp = 0;
  private Field2d field;

  static PoseEstimator estimator = new PoseEstimator();
  
  public PoseEstimator() {
    swerve = Swerve.getInstance();
    visionSubsystem = Vision.getVisionInstance();
    field = new Field2d();
    SmartDashboard.putData("field",field);
       try{
          layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile);
        }
        catch(IOException e) {
                DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
                layout = null;
        }
    drivePoseEstimator = new SwerveDrivePoseEstimator(
      Constants.swerveKinematics, 
      swerve.getPigeonAngle(), 
      swerve.getSwervePoses(), 
      new Pose2d()
    );
    photonEstimator = visionSubsystem.getVisionPose();
}

  public static PoseEstimator getPoseEstimatorInstance() {
    if (estimator == null) {
      estimator = new PoseEstimator();
    }
    return estimator;
  }

public void updatePoseEstimator() {
  if(visionSubsystem.getApriltagCamera().getLatestResult().hasTargets()) {
    photonEstimator.update(visionSubsystem.getPipelineResult()).ifPresent(estimatedRobotPose -> {
     var estimatedPose = estimatedRobotPose.estimatedPose;
    

     // m_DrivePoseEstimator.addVisionMeasurement(estimatedPose.toPose2d(), FIELD_LENGTH_METERS);
    
     // Make sure we have a new measurement, and that it's on the field
     if (visionSubsystem.getApriltagCamera().getLatestResult().getBestTarget().getFiducialId() >= 0){
      
     if (
       estimatedRobotPose.timestampSeconds != previousPipelineTimestamp && 
     estimatedPose.getX() >= 0.0 && estimatedPose.getX() <= FIELD_LENGTH_METERS
     && estimatedPose.getY() >= 0.0 && estimatedPose.getY() <= FIELD_WIDTH_METERS) {
       if (estimatedRobotPose.targetsUsed.size() >= 1) {
       
         for (PhotonTrackedTarget target : estimatedRobotPose.targetsUsed) {
           Pose3d targetPose = visionSubsystem.return_tag_pose(target.getFiducialId());
           Transform3d bestTarget = target.getBestCameraToTarget();
           Pose3d camPose;
           //Adding vision measurements from the center of the robot to the apriltag. Back camera should already be inverted
           // camPose = targetPose.transformBy(bestTarget.inverse().plus(visionSubsystem.getRobotToCam().inverse()));  //.plus(new Transform3d(robotToCam, new Rotation3d(0,0,0))); 
           camPose = targetPose.transformBy(bestTarget.inverse().plus(new Transform3d(new Translation3d(0.0,0.0,0.0), new Rotation3d(0,Units.degreesToRadians(35),Math.PI))));  //.plus(new Transform3d(robotToCam, new Rotation3d(0,0,0))); 

         //checking the tags ambiguity. The lower the ambiguity, the more accurate the pose is
           if (target.getPoseAmbiguity() <= .2) {
             previousPipelineTimestamp = estimatedRobotPose.timestampSeconds;
             drivePoseEstimator.addVisionMeasurement(camPose.toPose2d(), estimatedRobotPose.timestampSeconds);
           }
         }
       } 
     }

       else {
           previousPipelineTimestamp = estimatedRobotPose.timestampSeconds;
           drivePoseEstimator.addVisionMeasurement(estimatedPose.toPose2d(), estimatedRobotPose.timestampSeconds);
       }
     }
     }
     );
   }

  }

  @Override
  public void periodic() {
    drivePoseEstimator.update(swerve.getPigeonAngle(), swerve.getSwervePoses());
    //updatePose(drivePoseEstimator);
    SmartDashboard.putNumber("X", getPose().getX());
    SmartDashboard.putNumber("Y", getPose().getY());
    updatePoseEstimator();
    field.setRobotPose(getPose());
    //System.out.println(visionSubsystem.getPipelineResult());
  }


  public Pose2d getPose() {
    return drivePoseEstimator.getEstimatedPosition();
  }

 public void setCurrentPose(Pose2d newPose) {
   drivePoseEstimator.resetPosition(swerve.getPigeonAngle(), swerve.getSwervePoses(), newPose);
 }

}