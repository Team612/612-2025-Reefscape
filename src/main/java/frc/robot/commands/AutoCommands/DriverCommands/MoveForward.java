// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.AutoCommands.DriverCommands;

// import java.io.IOException;
// import java.util.List;

// import org.json.simple.parser.ParseException;
// import org.photonvision.PhotonPoseEstimator.PoseStrategy;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.path.GoalEndState;
// import com.pathplanner.lib.path.GoalEndState;
// import com.pathplanner.lib.path.PathConstraints;
// import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.path.Waypoint;
// import com.pathplanner.lib.util.FileVersionException;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import frc.robot.Constants;
// import frc.robot.subsystems.Mecanum;
// import frc.robot.subsystems.PoseEstimator;
// import frc.robot.subsystems.Vision;
// import frc.robot.util.TrajectoryCreation;

// public class MoveForward extends Command {
//   private final Mecanum driveSystem;
//   private final Vision m_vision;
//   private final PoseEstimator poseEstimatorSystem;
//   private final TrajectoryCreation m_traj;
//   private final double translation;
//       private PathConstraints constraints;
//   private final boolean bool;

//   private Command controllerCommand = Commands.none();

//   /** Creates a new RunOnTheFly. */
//   public MoveForward(Mecanum d, PoseEstimator p, TrajectoryCreation traj, Vision v, 
//                     double y, boolean b) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     driveSystem = d;
//     poseEstimatorSystem = p;
//     m_traj = traj;
//     m_vision = v;
//     translation = y;
//     bool = b;

//     constraints = new PathConstraints(Constants.AutoConstants.maxVelocity,
//     Constants.AutoConstants.maxAcceleration,
//      Constants.AutoConstants.maxAngularVelocity,
//       Constants.AutoConstants.maxAngularAcceleration);


//     addRequirements(d, v, p);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//    // PathPlannerPath path = m_traj.onthefly(poseEstimatorSystem, m_vision, translation);
//     //PathPlannerPath path = m_traj.apriltagCentering(poseEstimatorSystem, m_vision);

//     PathPlannerPath path = null;
//     try {
//       path = m_traj.ForwardMeter(poseEstimatorSystem);
//     } catch (Exception e) {
//       e.printStackTrace();
//     }
//     // var path = m_traj.apriltagCentering(poseEstimatorSystem, m_vision);\

//     path.preventFlipping = true;
//     if (path != null) {
//       controllerCommand = AutoBuilder.followPath(path);
//       controllerCommand.initialize();
//     } else {
//       controllerCommand = new Command() {
//         @Override
//         public boolean isFinished() {
//           return true;
//         }
//       };
//     }
//     //controllerCommand = AutoBuilder.followPath(path);
//   }


//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//       controllerCommand.execute();
    
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     controllerCommand.end(interrupted);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return controllerCommand.isFinished();
    
//   }
// }