// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import java.io.IOException;
// import java.util.Optional;

// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonPoseEstimator;
// import org.photonvision.PhotonPoseEstimator.PoseStrategy;
// import org.photonvision.targeting.PhotonPipelineResult;

// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.apriltag.AprilTagFields;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// // import frc.robot.Constants;
// import frc.robot.Constants;
// public class Vision extends SubsystemBase {
//   private static AprilTagFieldLayout aprilTagFieldLayout;
//   private Mecanum driveSubsystem;
//   private PhotonPoseEstimator photonPoseEstimator;
//   private static Vision visionInstance = null;

//   private Transform3d cameraOffset;

//   PhotonCamera camera;


//   /**
//    * Creates a new Vision.
//    * 
//    * @throws IOException
//    **/

//   public Vision() {
    
//     //creates camera instance
//     camera = new PhotonCamera("Limelight"); 

//     cameraOffset = new Transform3d(Constants.trackWidth/2,0,0,new Rotation3d());

//     //loads apriltag JSON file
//     aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape); 

//     //instantiates photon pose estimator
//     photonPoseEstimator = new PhotonPoseEstimator(
//       aprilTagFieldLayout,
//       PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
//      cameraOffset); 

//     driveSubsystem = Mecanum.getInstance();

  


//   }

//   public static Vision getVisionInstance() {
//     if (visionInstance == null) {
//       visionInstance = new Vision();
//     }
//     return visionInstance;
//   }


//   //return: the photon pose estimator
//   public PhotonPoseEstimator getVisionPose(){
//     return photonPoseEstimator;
//   }

//   //return: photon camera
//   public PhotonCamera getApriltagCamera(){
//     return camera; 
//   }

//   //return: camera transformation
//   public Transform3d getRobotToCam(){
//     return cameraOffset; //center of robot to camera transformation
//   }

//   //checks if camera is callibrated with the right coefficients
//   public boolean hasCalibration(){ 
//     if (camera.getDistCoeffs().equals(Optional.empty())){
//       return false;
//     }
//     return true;
//   }

//   //returns true if an april tag is detect, false otherwise
//   public boolean hasTag(){ 
//     if (camera.getLatestResult().hasTargets()){
//       if (camera.getLatestResult().getBestTarget().getFiducialId() >= 0){
//         return true;
//       }
//     }
//     return false;
//   }

  
//   // getting the vision pose from the april tags
//   // public Pose2d getTagPose() { //pose of the april tag detected
//   //   PhotonPipelineResult result = camera.getLatestResult();
//   //   if (result.hasTargets()) {
//   //     PhotonTrackedTarget bestTarget = result.getBestTarget();

//   //     Transform3d tagSpace = bestTarget.getBestCameraToTarget();

//   //     return new Pose2d(tagSpace.getX(), tagSpace.getY(), new Rotation2d( (bestTarget.getYaw()) * (Math.PI/180)) ); //imports the 3d transform to a 2d
//   //   }
//   //   return new Pose2d();
//   // }


//   // return: FIELD RELATIVE tag pose
//   public Pose3d return_tag_pose(int id) {
//     Optional<Pose3d> pose_of_tag = aprilTagFieldLayout.getTagPose(id);
//     return pose_of_tag.get();
//   }

//   //  return: estimated postion of the robot
//   public Pose3d return_camera_pose_tag(PhotonPipelineResult results) {
//     //return the fidual ID
//     if(results.hasTargets()){
//       int id = results.getBestTarget().getFiducialId();
//       //get the FIELD RELATIVE position of the tag
//       Optional<Pose3d> pose_of_tag = aprilTagFieldLayout.getTagPose(id);
//       Pose3d tag_pose = pose_of_tag.get();
//       //gets the ROBOT RELATIVE distance away from the tag
//       Transform3d cameraTransform = results.getBestTarget().getBestCameraToTarget();
//       //adds the ROBOT RELATIVE distance to the FIELD RELATIVE coordiantes, thus estimating robot position
//       return tag_pose.plus(cameraTransform.inverse());
//     }

//     return new Pose3d();
//   }



//   @Override
//   public void periodic() {
//     if (camera.getDistCoeffs().equals(Optional.empty())){
//       System.out.println("NO CALIBRATION");
//     }

//   }

  
// }