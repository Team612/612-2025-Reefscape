// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.proto.Photon;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTagFields;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
import frc.robot.Constants;
public class Vision extends SubsystemBase {
  private static AprilTagFieldLayout aprilTagFieldLayout;
  private Swerve driveSubsystem;
  private PhotonPoseEstimator photonPoseEstimator;

  private static Vision visionInstance = null;

  PhotonCamera camera;

  private Pose2d robotInTagPose;
  /**
   * Creates a new Vision.
   * 
   * @throws IOException
   **/

  public Vision() {
    
    camera = new PhotonCamera("Limelight"); //new camera instance
    

    resetRobotPose();

    aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape); //how to load the april tags from the field

    photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d(Constants.trackWidth/2,0,0,new Rotation3d())); //instantiates photon pose estimator

    driveSubsystem = Swerve.getInstance();

    //m_PoseEstimatorFront.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

  }

  public static Vision getVisionInstance() {
    if (visionInstance == null) {
      visionInstance = new Vision();
    }
    return visionInstance;
  }

  //  public PhotonPoseEstimator getVisionPoseFront(){
  //   return poseEstimatorFront;
  // }

  public PhotonPoseEstimator getVisionPose(){
    return photonPoseEstimator;
  }

  public PhotonCamera getApriltagCamera(){
    return camera; //there is only one camera
  }

  public Transform3d getRobotToCam(){
    return new Transform3d(); //center of robot to camera transformation
  }

  public boolean hasCalibration(){ //checks if camera is callibrated with the right coefficients
    if (camera.getDistCoeffs().equals(Optional.empty())){
      return false;
    }
    return true;
  }

  public boolean hasTag(){ //if it detects an april tag
    if (camera.getLatestResult().hasTargets()){
      if (camera.getLatestResult().getBestTarget().getFiducialId() >= 0){
        return true;
      }
    }
    return false;
  }

  public int tagID(){ //returns the ID of the apriltag ift detects, only runs if hasTag and has targets
     if (camera.getLatestResult().hasTargets()){
      return camera.getLatestResult().getBestTarget().getFiducialId();
    }
    return -1;
  }

  public PhotonPipelineResult getPipelineResult() {
    if (camera.getLatestResult().hasTargets()) {
      return camera.getLatestResult();
    }
    return new PhotonPipelineResult();
  }  

  
  // getting the vision pose from the april tags
  public Pose2d getTagPose() { //pose of the april tag detected
    PhotonPipelineResult result = camera.getLatestResult();
    if (result.hasTargets()) {
      PhotonTrackedTarget bestTarget = result.getBestTarget();

      Transform3d tagSpace = bestTarget.getBestCameraToTarget();

      return new Pose2d(tagSpace.getX(), tagSpace.getY(), new Rotation2d( (bestTarget.getYaw()) * (Math.PI/180)) ); //imports the 3d transform to a 2d
    }
    return new Pose2d();
  }

  public Pose2d getRobotPose(){
    Pose2d tagPose = robotInTagPose;
    return new Pose2d().transformBy(new Transform2d(tagPose.getTranslation(), tagPose.getRotation()));
  }

  public void resetRobotPose(){
    robotInTagPose = getTagPose();
  }

  // return tag pose
  public Pose3d return_tag_pose(int id) {
    Optional<Pose3d> pose_of_tag = aprilTagFieldLayout.getTagPose(id);
    return pose_of_tag.get();
  }

  // self calculations
  public Pose3d return_camera_pose_tag(int id, PhotonPipelineResult results) {
    Optional<Pose3d> pose_of_tag = aprilTagFieldLayout.getTagPose(id);
    Pose3d tag_pose = pose_of_tag.get();
    Transform3d cameraTransform = results.getBestTarget().getBestCameraToTarget();
    return tag_pose.plus(cameraTransform);
  }

  // photonvision pose estimator
  public Optional<EstimatedRobotPose> return_photon_pose(Pose2d latestPose) {
    photonPoseEstimator.setReferencePose(latestPose);
    return photonPoseEstimator.update(new PhotonPipelineResult());
  }


  @Override
  public void periodic() {
    // if (cameraApriltagBack.getDistCoeffs().equals(Optional.empty())){
    //   System.out.println("NO CALIBRATION");
    // }
    // if (hasTarget()){
    //   SmartDashboard.putNumber("note x", getNoteSpace().getX());
    //   SmartDashboard.putNumber("note y", getNoteSpace().getY());
    // }
    // SmartDashboard.putBoolean("Sees tag", cameraObject.getLatestResult().hasTargets());

    // if (cameraObject != null && cameraObject.getLatestResult().hasTargets()){
    //   SmartDashboard.putBoolean("has Object", cameraObject.getLatestResult().hasTargets());
    // }

  }

  
}