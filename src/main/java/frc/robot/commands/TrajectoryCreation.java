// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TrajectoryCreation extends Command {
  private PathConstraints constraints = new PathConstraints(Constants.maxSpeed, Constants.maxAcceleration, Constants.maxAngularVelocity, Constants.maxAngularAcceleration);
  public PathPlannerPath StrafeRight(PoseEstimator estimation, double distance){ 
        Pose2d estimatedPose = estimation.getCurrentPose();
        double x = estimatedPose.getX();
        double y = estimatedPose.getY();
        Rotation2d angle = estimatedPose.getRotation();

        List<Waypoint> bezierPoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(x, y, angle),
            new Pose2d(x, y + distance, angle)
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
    public PathPlannerPath StrafeLeft(PoseEstimator estimation, double distance){ 
      Pose2d estimatedPose = estimation.getCurrentPose();
      double x = estimatedPose.getX();
      double y = estimatedPose.getY();
      Rotation2d angle = estimatedPose.getRotation();

      List<Waypoint> bezierPoints = PathPlannerPath.waypointsFromPoses(
          new Pose2d(x, y, angle),
          new Pose2d(x, y + distance, angle)
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
      public PathPlannerPath Forward(PoseEstimator estimation, double distance){
        Pose2d estimatedPose = estimation.getCurrentPose();

        double x = estimatedPose.getX();
        double y = estimatedPose.getY();
        Rotation2d angle = estimatedPose.getRotation();
        System.out.println();

        List<Waypoint> bezierPoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(x, y, new Rotation2d(-90)),
            new Pose2d(x, y-distance, new Rotation2d(-90))
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
      public PathPlannerPath Backward(PoseEstimator estimation, double distance){
        Pose2d estimatedPose = estimation.getCurrentPose();

        double x = estimatedPose.getX();
        double y = estimatedPose.getY();
        Rotation2d angle = estimatedPose.getRotation();

        List<Waypoint> bezierPoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(x, y, angle),
            new Pose2d(x, y+distance, angle)
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

    public Pose2d getApriltagPose(PoseEstimator estimation, Vision vision){
        var pipeline = vision.getFrontApriltagCamera().getAllUnreadResults();
        if(!pipeline.isEmpty()){
            PhotonPipelineResult result = vision.getFrontPipelineResult();
            Translation3d tag = vision.return_tag_pose(result.getBestTarget().getFiducialId()).getTranslation();
            double tagX = tag.getX();
            double tagY = tag.getY();
            Rotation2d tagAngle = vision.return_tag_pose(result.getBestTarget().getFiducialId()).getRotation().toRotation2d().rotateBy(new Rotation2d(Units.degreesToRadians(180)));
    
            return new Pose2d(tagX - (Math.cos(tagAngle.getRadians())), tagY - (Math.sin(tagAngle.getRadians())), tagAngle);
        }

        else{
            System.out.println("TRUE");
          return null;
        }
    }

    public PathPlannerPath apriltagCentering(PoseEstimator estimation, Vision vision){
    
        Pose2d estimatedPose = estimation.getCurrentPose();
        double x = estimatedPose.getX();
        double y = estimatedPose.getY();
        Rotation2d angle = estimatedPose.getRotation();

        PhotonPipelineResult result = vision.getFrontApriltagCamera().getAllUnreadResults().get(0);

        Translation3d tag = vision.return_tag_pose(result.getBestTarget().getFiducialId()).getTranslation();
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
        Pose2d finalPose = new Pose2d(tagX, tagY, tagAngle).transformBy(new Transform2d(new Translation2d(-1,0),new Rotation2d()));
        



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

    public PathPlannerPath onthefly(PoseEstimator estimation, Vision vision, double y_translation){
        Pose2d estimatedPose = estimation.getCurrentPose();
        double x = estimatedPose.getX();
        double y = estimatedPose.getY();
        Rotation2d angle = estimatedPose.getRotation();
       
        PhotonPipelineResult result = vision.getFrontApriltagCamera().getLatestResult();
        int id;
        double tagX = 0;
        double tagY = 0; 
        Rotation2d tagAngle = new Rotation2d();

        if(result.hasTargets()){
            id = vision.getFrontApriltagCamera().getLatestResult().getBestTarget().getFiducialId();


            Pose2d tagPose = vision.return_tag_pose(id).toPose2d();
            tagX = tagPose.getX();
            tagY = tagPose.getY();
            tagAngle = new Rotation2d(Math.PI - tagPose.getRotation().getRadians());
        }
        else{
            id = -1;
        }

        double offset =0;
        
        if(id == 9 || id == 19) {
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
        } else if(id == 5 || id == 4 || id == 21 || id == 7) {
            List<Waypoint> bezierPoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(x, y, angle),
                new Pose2d(tagX - 1, tagY - offset, tagAngle)
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
        } else if(id == 3) {
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
      }else if(id == 16) {
            List<Waypoint> bezierPoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(x, y, angle),
                new Pose2d(tagX + offset, tagY + 1, tagAngle)
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
        } else if(id == 15 || id == 14) {
            List<Waypoint> bezierPoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(x, y, angle),
                new Pose2d(tagX + 1, tagY - offset, new Rotation2d(Math.PI))
            );

            // Create the path using the bezier points created above
            PathPlannerPath path = new PathPlannerPath(
                bezierPoints,
                constraints,
                null,
                new GoalEndState(0.0, new Rotation2d(Math.PI)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
            );

            // Prevent the path from being flipped if the coordinates are already correct
            path.preventFlipping = true;
            return path;
        } else if(id == 8 || id == 20) {
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
        } else if(id == 6 || id == 22) {
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
        } else if(id == 11 || id == 17) {
            List<Waypoint> bezierPoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(x, y, angle),
                new Pose2d(tagX - 0.5, tagY - 0.866, tagAngle)
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
        } else if(id == 12) {
          List<Waypoint> bezierPoints = PathPlannerPath.waypointsFromPoses(
              new Pose2d(x, y, angle),
              new Pose2d(tagX + Math.cos(Units.degreesToRadians(54)), tagY + Math.sin(Units.degreesToRadians(54)), tagAngle)
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
      }else if(id == 12) {
          List<Waypoint> bezierPoints = PathPlannerPath.waypointsFromPoses(
              new Pose2d(x, y, angle),
              new Pose2d(tagX + Math.cos(Units.degreesToRadians(54)), tagY + Math.sin(Units.degreesToRadians(54)), tagAngle)
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
      } else if(id == 1) {
          List<Waypoint> bezierPoints = PathPlannerPath.waypointsFromPoses(
              new Pose2d(x, y, angle),
              new Pose2d(tagX + Math.cos(Units.degreesToRadians(126)), tagY + Math.sin(Units.degreesToRadians(126)), tagAngle)
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
      } else if(id == 2) {
          List<Waypoint> bezierPoints = PathPlannerPath.waypointsFromPoses(
              new Pose2d(x, y, angle),
              new Pose2d(tagX + Math.cos(Units.degreesToRadians(234)), tagY + Math.sin(Units.degreesToRadians(234)), tagAngle)
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
      } else if(id == 13) {
          List<Waypoint> bezierPoints = PathPlannerPath.waypointsFromPoses(
              new Pose2d(x, y, angle),
              new Pose2d(tagX + Math.cos(Units.degreesToRadians(306)), tagY + Math.sin(Units.degreesToRadians(306)), tagAngle)
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
      }else {
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
  /** Creates a new TrajectoryCreation. */
  public TrajectoryCreation() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
