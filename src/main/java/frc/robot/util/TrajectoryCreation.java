package frc.robot.util;

import java.util.List;

import org.photonvision.targeting.PhotonPipelineResult;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Vision;

public class TrajectoryCreation {
    private static TrajectoryCreation instance;
    private PathConstraints constraints = new PathConstraints(Constants.AutoConstants.maxVelocity,
     Constants.AutoConstants.maxAcceleration,
      Constants.AutoConstants.maxAngularVelocity,
       Constants.AutoConstants.maxAngularAcceleration);

    public PathPlannerPath StrafeRightMeter(PoseEstimator estimation){ 
        Pose2d estimatedPose = estimation.getPose();
        double x = estimatedPose.getX();
        double y = estimatedPose.getY();
        Rotation2d angle = estimatedPose.getRotation();

        List<Waypoint> bezierPoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(x, y, angle),
            new Pose2d(x, y + 1, angle)
        );

        // Create the path using the bezier points created above
        PathPlannerPath path = new PathPlannerPath(
            bezierPoints,
            constraints,
            null,
            new GoalEndState(0.0, angle) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );

        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping = true;
        return path;
    }

    public PathPlannerPath StrafeLeftMeter(PoseEstimator estimation){ 
        Pose2d estimatedPose = estimation.getPose();
        double x = estimatedPose.getX();
        double y = estimatedPose.getY();
        Rotation2d angle = estimatedPose.getRotation();

        List<Waypoint> bezierPoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(x, y, angle),
            new Pose2d(x, y + 1, angle)
        );

        // Create the path using the bezier points created above
        PathPlannerPath path = new PathPlannerPath(
            bezierPoints,
            constraints,
            null,
            new GoalEndState(0.0, angle) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );

        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping = true;
        return path;
    }

    public PathPlannerPath ForwardMeter(PoseEstimator estimation){
        Pose2d estimatedPose = estimation.getPose();

        double x = estimatedPose.getX();
        double y = estimatedPose.getY();
        Rotation2d angle = estimatedPose.getRotation();
        System.out.println();

        List<Waypoint> bezierPoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(x, y, angle),
            new Pose2d(x, y-(1), angle)
        );

        // Create the path using the bezier points created above
        PathPlannerPath path = new PathPlannerPath(
            bezierPoints,
            constraints,
            null,
            new GoalEndState(0.0, angle) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );

        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping = true;
        return path;
    }

    public PathPlannerPath BackwardMeter(PoseEstimator estimation){
        Pose2d estimatedPose = estimation.getPose();

        double x = estimatedPose.getX();
        double y = estimatedPose.getY();
        Rotation2d angle = estimatedPose.getRotation();

        List<Waypoint> bezierPoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(x, y, angle),
            new Pose2d(x, y+1, angle)
        );
        System.out.println(x + " " + y+1);

        // Create the path using the bezier points created above
        PathPlannerPath path = new PathPlannerPath(
            bezierPoints,
            constraints,
            null,
            new GoalEndState(0.0, angle) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );

        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping = true;
        return path;
    }

    // public Pose2d getApriltagPose(PoseEstimator estimation, Vision vision){
    //     var pipeline = vision.getFrontPipelineResult();
    //     if(!pipeline.isEmpty()){
    //         PhotonPipelineResult result = pipeline.get(0);


    //         Translation3d tag = vision.return_tag_pose(result.getBestTarget().getFiducialId()).getTranslation();
    //         double tagX = tag.getX();
    //         double tagY = tag.getY();
    //         Rotation2d tagAngle = vision.return_tag_pose(result.getBestTarget().getFiducialId()).getRotation().toRotation2d().rotateBy(new Rotation2d(Units.degreesToRadians(180)));
    
    //         return new Pose2d(tagX - (Math.cos(tagAngle.getRadians())), tagY - (Math.sin(tagAngle.getRadians())), tagAngle);
    //     }

    //     else{
    //         System.out.println("TRUE");
    //       return null;
    //     }
    // }

    public PathPlannerPath apriltagCentering(PoseEstimator estimation, Vision vision, double displacementX, double displacementY){
    
        Pose2d estimatedPose = estimation.getPose();
        double x = estimatedPose.getX();
        double y = estimatedPose.getY();
        Rotation2d angle = estimatedPose.getRotation();

        PhotonPipelineResult result = vision.getFrontPipelineResult();
        System.out.println(result);
        System.out.println(result.getBestTarget());

        
        double tagX = vision.return_tag_pose(result.getBestTarget().getFiducialId()).getX();
        double tagY = vision.return_tag_pose(result.getBestTarget().getFiducialId()).getY();
        SmartDashboard.putNumber("Y:", tagY);
        SmartDashboard.putNumber("X", tagX);


        //gets the angle RELATIVE to the field through polar coordinates
        Rotation2d heading = new Translation2d(tagX - x, tagY - y).getAngle(); 
        //gets the tag angle, flipped 180 for field relative
        Rotation2d tagAngle = vision.return_tag_pose(result.getBestTarget().getFiducialId()).getRotation().toRotation2d().rotateBy(new Rotation2d(Units.degreesToRadians(180)));
        //final pose is the tag pose transformed by a certain distance RELATIVE to the tag (with robot coordinates)
        //note that the heading should be the tag angle so it faces outward. Putting the tag angle as the heading results in trajectory heading towards the tag, not infront
        Pose2d midPose1 = new Pose2d(tagX, tagY, tagAngle).transformBy(new Transform2d(new Translation2d(displacementX-1,displacementY),new Rotation2d()));
        Pose2d midPose2 = new Pose2d(tagX, tagY, tagAngle).transformBy(new Transform2d(new Translation2d(displacementX-1,displacementY+Units.inchesToMeters(12.947811)),new Rotation2d()));
        Pose2d finalPose = new Pose2d(tagX, tagY, tagAngle).transformBy(new Transform2d(new Translation2d(displacementX,displacementY),new Rotation2d()));
        
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(x,y, heading),
            midPose1,
            midPose2,
            finalPose
        
        );

        // Create the path using the bezier points created above
        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            constraints,
            null,
            new GoalEndState(0.0, tagAngle) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );

        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping = true;
        return path;
    }

    public PathPlannerPath coralStationCentering(PoseEstimator estimation, Vision vision){
    
        Pose2d estimatedPose = estimation.getPose();
        double x = estimatedPose.getX();
        double y = estimatedPose.getY();
        Rotation2d angle = estimatedPose.getRotation();

        double displacementX = Units.inchesToMeters(16);
        double displacementY = 0;


        PhotonPipelineResult result = vision.getBackPipelineResult();
        System.out.println(result);
        System.out.println(result.getBestTarget());

        
        double tagX = vision.return_tag_pose(result.getBestTarget().getFiducialId()).getX();
        double tagY = vision.return_tag_pose(result.getBestTarget().getFiducialId()).getY();
        SmartDashboard.putNumber("Y:", tagY);
        SmartDashboard.putNumber("X", tagX);


        //gets the angle RELATIVE to the field through polar coordinates
        Rotation2d heading = new Translation2d(tagX - x, tagY - y).getAngle(); 
        //gets the tag angle, flipped 180 for field relative
        Rotation2d tagAngle = vision.return_tag_pose(result.getBestTarget().getFiducialId()).getRotation().toRotation2d().rotateBy(new Rotation2d(Units.degreesToRadians(0)));
        //final pose is the tag pose transformed by a certain distance RELATIVE to the tag (with robot coordinates)
        //note that the heading should be the tag angle so it faces outward. Putting the tag angle as the heading results in trajectory heading towards the tag, not infront
        Pose2d finalPose = new Pose2d(tagX, tagY, tagAngle).transformBy(new Transform2d(new Translation2d(displacementX,displacementY),new Rotation2d()));
        
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(x,y, heading),
            finalPose
        
        );

        // Create the path using the bezier points created above
        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            constraints,
            null,
            new GoalEndState(0.0, tagAngle) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );

        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping = true;
        return path;
    }

    public static TrajectoryCreation getInstance(){
        if (instance == null){
            instance = new TrajectoryCreation();
        }
        return instance;
    }

    
}