// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Vision extends SubsystemBase {
  private static AprilTagFieldLayout aprilTagFieldLayout;
  private Mecanum driveSubsystem;
  private PhotonPoseEstimator photonFrontPoseEstimator;
  private PhotonPoseEstimator photonBackPoseEstimator;

  private static Vision visionInstance = null;

  PhotonCamera frontCamera;
  PhotonCamera backCamera;

  private Pose2d robotInTagPose;
  /**
   * Creates a new Vision.
   * 
   * @throws IOException
   **/

  public Vision() {
    
    frontCamera = new PhotonCamera("FrontCamera"); //new camera instance
    backCamera = new PhotonCamera("BackCamera");
    

    aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark); //how to load the april tags from the field

    photonFrontPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d()); //instantiates photon pose estimator
    photonBackPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d());

    driveSubsystem = Mecanum.getInstance();

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

  public PhotonPoseEstimator getFrontVisionEstimator(){
    return photonFrontPoseEstimator;
  }
  public PhotonPoseEstimator getBackVisionEStimator(){
    return photonBackPoseEstimator;
  }

  public PhotonCamera getFrontApriltagCamera(){
    return frontCamera; 
  }
  public PhotonCamera getBackApriltagCamera(){
    return backCamera;
  }

  public Transform3d getRobotToCam(){
    return new Transform3d(); //center of robot to camera transformation
  }

  public boolean hasCalibration(){ //checks if camera is callibrated with the right coefficients
    if (frontCamera.getDistCoeffs().equals(Optional.empty())){
      return false;
    }
    return true;
  }

  public boolean frontHasTag(){ //if it detects an april tag
    if (frontCamera.getLatestResult().hasTargets()){
      if (frontCamera.getLatestResult().getBestTarget().getFiducialId() >= 0){
        return true;
      }
    }
    return false;
  }
  public boolean backHasTag(){ //if it detects an april tag
    if (backCamera.getLatestResult().hasTargets()){
      if (backCamera.getLatestResult().getBestTarget().getFiducialId() >= 0){
        return true;
      }
    }
    return false;
  }
  public int frontTagID(){ //returns the ID of the apriltag ift detects, only runs if hasTag and has targets
     if (frontCamera.getLatestResult().hasTargets()){
      return frontCamera.getLatestResult().getBestTarget().getFiducialId();
    }
    return -1;
  }
  public int backTagID(){ //returns the ID of the apriltag ift detects, only runs if hasTag and has targets
     if (backCamera.getLatestResult().hasTargets()){
      return backCamera.getLatestResult().getBestTarget().getFiducialId();
    }
    return -1;
  }

  public PhotonPipelineResult getFrontPipelineResult() {
    if (frontCamera.getLatestResult().hasTargets()) {
      return frontCamera.getLatestResult();
    }
    return new PhotonPipelineResult();
  }  

  public PhotonPipelineResult getBackPipelineResult() {
    if (backCamera.getLatestResult().hasTargets()) {
      return backCamera.getLatestResult();
    }
    return new PhotonPipelineResult();
  }  
  
  // getting the vision pose from the april tags
  public Pose2d getTagPose(PhotonPipelineResult result) { //pose of the april tag detected
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
    photonFrontPoseEstimator.setReferencePose(latestPose);
    return photonFrontPoseEstimator.update(new PhotonPipelineResult());
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Tag X", getTagPose(frontCamera.getLatestResult()).getX());
    SmartDashboard.putNumber("Tag Y", getTagPose(frontCamera.getLatestResult()).getY());

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
    // }        PhotonPipelineResult result = vision.getApriltagCamera().getAllUnreadResults().get(0);

    // PhotonPipelineResult result = getVisionInstance().getApriltagCamera().getLatestResult();

    // Translation3d tag = getVisionInstance().return_tag_pose(result.getBestTarget().getFiducialId()).getTranslation();
    // double tagX = getVisionInstance().return_tag_pose(result.getBestTarget().getFiducialId()).getX();
    // double tagY = getVisionInstance().return_tag_pose(result.getBestTarget().getFiducialId()).getY();
    // SmartDashboard.putNumber("Y:", tagY);
    // SmartDashboard.putNumber("X", tagX);


  }

  
}