// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;


// import java.io.IOException;
// import java.util.Optional;

// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
// import edu.wpi.first.apriltag.AprilTagFields;
// import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.Vector;
// import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
// import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.geometry.Translation3d;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import org.photonvision.*;
// import org.photonvision.targeting.PhotonPipelineResult;
// import org.photonvision.targeting.PhotonTrackedTarget;


// public class PoseEstimator extends SubsystemBase {

//   Vision visionSubsystem;
//   Mecanum mech;

//   Pose3d estimatedPose;

//   AprilTagFieldLayout layout;
//   MecanumDrivePoseEstimator drivePoseEstimator;
//   PhotonPoseEstimator photonEstimator;
//   private Field2d field;

//   static PoseEstimator estimator = new PoseEstimator();
  
//   public PoseEstimator() {
//     mech = Mecanum.getInstance();
//     visionSubsystem = Vision.getVisionInstance();
//     field = new Field2d();
//     SmartDashboard.putData("field",field);

//        try{
//           layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile);
//         }
//         catch(IOException e) {
//                 DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
//                 layout = null;
//         }

//     drivePoseEstimator = new MecanumDrivePoseEstimator(Constants.DrivetrainConstants.m_kinematics, mech.getPigeonAngle(), mech.getMecanumDriveWheelPositions(), new Pose2d());

//     photonEstimator = visionSubsystem.getVisionPose();
// }

//   public static PoseEstimator getPoseEstimatorInstance() {
//     if (estimator == null) {
//       estimator = new PoseEstimator();
//     }
//     return estimator;
//   }

// public void updatePoseEstimator() {
//     //loops through all visible targets
//     if (!visionSubsystem.getApriltagCamera().getAllUnreadResults().isEmpty()){
//     for (PhotonPipelineResult tag : visionSubsystem.getApriltagCamera().getAllUnreadResults()){
//       if (tag.hasTargets()){
//       //returns the robot pose
//       Pose3d robotPose = visionSubsystem.return_camera_pose_tag(tag);
//       //adds vision measurements if ambiguity is less than 0.2
//       if (tag.getBestTarget().poseAmbiguity <= 0.2){
//         drivePoseEstimator.addVisionMeasurement(robotPose.toPose2d(), tag.getTimestampSeconds());
//       }    
//     }
//     }
//    //updates the pose estimator based off encoders.
//   }
//    drivePoseEstimator.update(mech.getPigeonAngle(), mech.getMecanumDriveWheelPositions());
//   }

//   @Override
//   public void periodic() {
//     updatePoseEstimator();
    
//     SmartDashboard.putNumber("X", getPose().getX());
//     SmartDashboard.putNumber("Y", getPose().getY());
//     SmartDashboard.putNumber("angle", getPose().getRotation().getDegrees());
    
//     field.setRobotPose(getPose());

//   }


//   public Pose2d getPose() {
//     return drivePoseEstimator.getEstimatedPosition();
//   }

//  public void setCurrentPose(Pose2d newPose) {
//    drivePoseEstimator.resetPosition(mech.getPigeonAngle(), mech.getMecanumDriveWheelPositions(), newPose);
//  }

// }