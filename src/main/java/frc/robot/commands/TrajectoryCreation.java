
package frc.robot.commands;

import java.util.List;

import org.photonvision.targeting.PhotonPipelineResult;

import com.pathplanner.lib.path.*;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Vision;

public class TrajectoryCreation {
    private PathConstraints constraints = new PathConstraints(Constants.maxSpeed,
     Constants.maxAcceleration,
      Constants.maxAngularVelocity,
       Constants.maxAngularAcceleration);

    public PathPlannerPath StrafeRightMeter(PoseEstimator estimation){ 
        Pose2d estimatedPose = estimation.getPose();

        double x = estimatedPose.getX();
        double y = estimatedPose.getY();
        Rotation2d angle = estimatedPose.getRotation();

        List<Waypoint> bezierPoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(x, y, angle),
            new Pose2d(x, y - 1, angle)
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

        List<Waypoint> bezierPoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(x, y, angle),
            new Pose2d(x + 1, y, angle)
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
            new Pose2d(x - 1, y, angle)
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

    public PathPlannerPath onthefly(PoseEstimator estimation, Vision vision, double y_translation){
        Pose2d estimatedPose = estimation.getPose();
        double x = estimatedPose.getX();
        double y = estimatedPose.getY();
        Rotation2d angle = estimatedPose.getRotation();
       
        PhotonPipelineResult result = vision.getApriltagCamera().getLatestResult();
        int id;
        double tagX = 0;
        double tagY = 0; 
        Rotation2d tagAngle = new Rotation2d();

        if(result.hasTargets()){
            id = vision.getApriltagCamera().getLatestResult().getBestTarget().getFiducialId();


            Pose2d tagPose = vision.return_tag_pose(id).toPose2d();
            tagX = tagPose.getX();
            tagY = tagPose.getY();
            tagAngle = new Rotation2d(-Units.degreesToRadians(180 - tagPose.getRotation().getDegrees()));
        }
        else{
            id = -1;
        }

        double offset = Constants.trackWidth / 2;
        
        if(id == 1 || id == 2 || id == 15) {
            List<Waypoint> bezierPoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(x, y, angle),
                new Pose2d(tagX - 0.5, tagY + 0.866 + offset, tagAngle)
            );
            System.out.println(tagX);
            System.out.println(tagY);
            System.out.println(tagAngle);

            // Create the path using the bezier points created above
            PathPlannerPath path = new PathPlannerPath(
                bezierPoints,
                constraints,
                null,
                new GoalEndState(0.0, tagAngle) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
            );

            // Prevent the path from being flipped if the coordinates are already correct
            path.preventFlipping = true;
            return path;
        } else if(id == 3 || id == 4 || id == 13) {
            List<Waypoint> bezierPoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(x, y, angle),
                new Pose2d(tagX - 2, tagY + offset, tagAngle)
            );

            // Create the path using the bezier points created above
            PathPlannerPath path = new PathPlannerPath(
                bezierPoints,
                constraints,
                null,
                new GoalEndState(0.0, tagAngle) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
            );

            // Prevent the path from being flipped if the coordinates are already correct
            path.preventFlipping = true;
            return path;
        } else if(id == 5 || id == 6) {
            List<Waypoint> bezierPoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(x, y, angle),
                new Pose2d(tagX - offset, tagY - 1, tagAngle)
            );

            // Create the path using the bezier points created above
            PathPlannerPath path = new PathPlannerPath(
                bezierPoints,
                constraints,
                null,
                new GoalEndState(0.0, tagAngle) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
            );

            // Prevent the path from being flipped if the coordinates are already correct
            path.preventFlipping = true;
            return path;
        } else if(id == 7 || id == 8 || id == 14) {
            List<Waypoint> bezierPoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(x, y, angle),
                new Pose2d(tagX + 1, tagY - offset, tagAngle)
            );

            // Create the path using the bezier points created above
            PathPlannerPath path = new PathPlannerPath(
                bezierPoints,
                constraints,
                null,
                new GoalEndState(0.0, tagAngle) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
            );

            // Prevent the path from being flipped if the coordinates are already correct
            path.preventFlipping = true;
            return path;
        } else if(id == 9 || id == 10 || id == 12) {
            List<Waypoint> bezierPoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(x, y, angle),
                new Pose2d(tagX + 0.5, tagY + 0.866, tagAngle)
            );

            // Create the path using the bezier points created above
            PathPlannerPath path = new PathPlannerPath(
                bezierPoints,
                constraints,
                null,
                new GoalEndState(0.0, tagAngle) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
            );

            // Prevent the path from being flipped if the coordinates are already correct
            path.preventFlipping = true;
            return path;
        } else if(id == 11) {
            List<Waypoint> bezierPoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(x, y, angle),
                new Pose2d(tagX + 0.5, tagY - 0.866, tagAngle)
            );

            // Create the path using the bezier points created above
            PathPlannerPath path = new PathPlannerPath(
                bezierPoints,
                constraints,
                null,
                new GoalEndState(0.0, tagAngle) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
            );

            // Prevent the path from being flipped if the coordinates are already correct
            path.preventFlipping = true;
            return path;
        } else if(id == 16) {
            List<Waypoint> bezierPoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(x, y, angle),
                new Pose2d(tagX - 0.5 - 0.433, tagY - 0.866 + 0.25, tagAngle)
            );

            // Create the path using the bezier points created above
            PathPlannerPath path = new PathPlannerPath(
                bezierPoints,
                constraints,
                null,
                new GoalEndState(0.0, tagAngle) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
            );

            // Prevent the path from being flipped if the coordinates are already correct
            path.preventFlipping = true;
            return path;
        } else {
            List<Waypoint> bezierPoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(x, y, angle),
                new Pose2d(tagX, tagY, angle)
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
    }   
}