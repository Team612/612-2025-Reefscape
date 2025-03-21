// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import java.io.IOException;

import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


// import frc.robot.LimelightHelpers;
// import frc.robot.LimelightHelpers.LimelightResults;
// import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
// import frc.robot.LimelightHelpers.PoseEstimate;
// import frc.robot.LimelightHelpers.RawFiducial;

public class PoseEstimator extends SubsystemBase {

  Vision visionSubsystem;
  Mecanum m_Mecanum;

  Pose3d estimatedPose;

  AprilTagFieldLayout layout;
  MecanumDrivePoseEstimator drivePoseEstimator;
  PhotonPoseEstimator photonEstimator;

  // StructPublisher<Pose2d> publisher;
  // StructArrayPublisher<Pose2d> arrayPublisher;
  // {
 
  // }

  public static final double FIELD_LENGTH_METERS = Units.inchesToMeters(690.875);
  public static final double FIELD_WIDTH_METERS = Units.inchesToMeters(317);
  private double previousPipelineTimestamp = 0;
  private Field2d field;
  private Pose3d poseA;
  private Pose3d poseB;
  private StructPublisher<Pose3d> publisher;
  private StructArrayPublisher<Pose3d> arrayPublisher;

  private static final Vector<N3> statesStdDev = VecBuilder.fill(0.5,0.5,1);
  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.05,0.05,0.01);

  static PoseEstimator estimator = new PoseEstimator();
  
  public PoseEstimator() {
    m_Mecanum = Mecanum.getInstance();
    visionSubsystem = Vision.getVisionInstance();
    field = new Field2d();
    SmartDashboard.putData("field",field);
       try{
          layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeAndyMark.m_resourceFile);
        }
        catch(IOException e) {
                DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
                layout = null;
        }
    drivePoseEstimator = new MecanumDrivePoseEstimator(
      Constants.DrivetrainConstants.m_kinematics, 
      m_Mecanum.getPigeonAngle(), 
      m_Mecanum.getMecanumDriveWheelPositions(), 
      new Pose2d(),
      statesStdDev,
      visionMeasurementStdDevs
    );
    photonEstimator = visionSubsystem.getPhotonPoseEstimator();


    poseA = new Pose3d();
    poseB = new Pose3d();

     publisher = NetworkTableInstance.getDefault()
      .getStructTopic("MyPose", Pose3d.struct).publish();
     arrayPublisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("MyPoseArray", Pose3d.struct).publish();
}

  public static PoseEstimator getPoseEstimatorInstance() {
    if (estimator == null) {
      estimator = new PoseEstimator();
    }
    return estimator;
  }

  public void updatePoseEstimator() {
    if (visionSubsystem.getFrontApriltagCamera() != null){
    if(visionSubsystem.frontHasTag() && visionSubsystem.backHasTag()){
        if(visionSubsystem.getFrontPipelineResult().getBestTarget().poseAmbiguity <= visionSubsystem.getBackPipelineResult().getBestTarget().poseAmbiguity){
        photonEstimator.update(visionSubsystem.getFrontPipelineResult()).ifPresent(estimatedRobotPose -> {
            var estimatedPose = estimatedRobotPose.estimatedPose;

            // Make sure we have a new measurement, and that it's on the field
            if (visionSubsystem.frontTagID() >= 0){
                if (estimatedRobotPose.timestampSeconds != previousPipelineTimestamp && 
                    estimatedPose.getX() >= 0.0 && estimatedPose.getX() <= FIELD_LENGTH_METERS &&
                    estimatedPose.getY() >= 0.0 && estimatedPose.getY() <= FIELD_WIDTH_METERS) {
                    if (estimatedRobotPose.targetsUsed.size() >= 1) {
                        for (PhotonTrackedTarget target : estimatedRobotPose.targetsUsed) {
                            Pose3d targetPose = visionSubsystem.return_tag_pose(target.getFiducialId());
                            Transform3d bestTarget = target.getBestCameraToTarget();
                            Pose3d camPose = targetPose.transformBy(bestTarget.plus(visionSubsystem.getRobotToFrontCam()).inverse());  //.plus(new Transform3d(robotToCam, new Rotation3d(0,0,0))); 
        
                            //checking the tags ambiguity. The lower the ambiguity, the more accurate the pose is
                            if (target.getPoseAmbiguity() <= .2) {
                                previousPipelineTimestamp = estimatedRobotPose.timestampSeconds;
                                drivePoseEstimator.addVisionMeasurement(camPose.toPose2d(), estimatedRobotPose.timestampSeconds);
                            }
                        }
                    } 
                }
              }
                else {
                    previousPipelineTimestamp = estimatedRobotPose.timestampSeconds;
                    drivePoseEstimator.addVisionMeasurement(estimatedPose.toPose2d(), estimatedRobotPose.timestampSeconds);
                }
            });
        }
    } 
    else if(visionSubsystem.backHasTag()) {
        photonEstimator.update(visionSubsystem.getBackPipelineResult()).ifPresent(estimatedRobotPose -> {
            var estimatedPose = estimatedRobotPose.estimatedPose;

            // Make sure we have a new measurement, and that it's on the field
            if (visionSubsystem.backTagID() >= 0) {
                if (estimatedRobotPose.timestampSeconds != previousPipelineTimestamp && 
                    estimatedPose.getX() >= 0.0 && estimatedPose.getX() <= FIELD_LENGTH_METERS &&
                    estimatedPose.getY() >= 0.0 && estimatedPose.getY() <= FIELD_WIDTH_METERS) {
                    if (estimatedRobotPose.targetsUsed.size() >= 1) {
                        for (PhotonTrackedTarget target : estimatedRobotPose.targetsUsed) {
                            Pose3d targetPose = visionSubsystem.return_tag_pose(target.getFiducialId());
                            Transform3d bestTarget = target.getBestCameraToTarget();
                            Pose3d camPose = targetPose.transformBy(bestTarget.inverse().plus(visionSubsystem.getRobotToBackCam()));  //.plus(new Transform3d(robotToCam, new Rotation3d(0,0,0))); 
        
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
        });
    } else if(visionSubsystem.frontHasTag()) {
      photonEstimator.update(visionSubsystem.getFrontPipelineResult()).ifPresent(estimatedRobotPose -> {
          var estimatedPose = estimatedRobotPose.estimatedPose;

          // Make sure we have a new measurement, and that it's on the field
          if (visionSubsystem.frontTagID() >= 0) {
              if (estimatedRobotPose.timestampSeconds != previousPipelineTimestamp && 
                  estimatedPose.getX() >= 0.0 && estimatedPose.getX() <= FIELD_LENGTH_METERS &&
                  estimatedPose.getY() >= 0.0 && estimatedPose.getY() <= FIELD_WIDTH_METERS) {
                  if (estimatedRobotPose.targetsUsed.size() >= 1) {
                      for (PhotonTrackedTarget target : estimatedRobotPose.targetsUsed) {
                          Pose3d targetPose = visionSubsystem.return_tag_pose(target.getFiducialId());
                          Transform3d bestTarget = target.getBestCameraToTarget();
                          Pose3d camPose = targetPose.transformBy(bestTarget.plus(visionSubsystem.getRobotToFrontCam()).inverse());  //.plus(new Transform3d(robotToCam, new Rotation3d(0,0,0))); 
      
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
      });
  }
  }
  }

  @Override
  public void periodic() {
    drivePoseEstimator.update(m_Mecanum.getPigeonAngle(), m_Mecanum.getMecanumDriveWheelPositions());
    updatePoseEstimator();
    field.setRobotPose(getPose());
    
    poseA = new Pose3d(getPose());
    publisher.set(poseA);
    arrayPublisher.set(new Pose3d[] {poseA, poseB});
  }

  public Pose2d getPose() {
    return drivePoseEstimator.getEstimatedPosition();
  }

 public void setCurrentPose(Pose2d newPose) {
   drivePoseEstimator.resetPosition(m_Mecanum.getPigeonAngle(), m_Mecanum.getMecanumDriveWheelPositions(), newPose);
 }

}