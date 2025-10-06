// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import frc.robot.subsystems.PoseEstimator;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.config.PIDConstants;
// import com.pathplanner.lib.config.RobotConfig;
// import com.pathplanner.lib.controllers.PPHolonomicDriveController;

// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.subsystems.Swerve;

// public class TrajectoryConfiguration extends SubsystemBase {
//   /** Creates a new TrajectoryConfiguration. */
//   private static TrajectoryConfiguration trajectoryConfig;
//   private PoseEstimator m_PoseEstimator = PoseEstimator.getPoseEstimatorInstance();
//   private Swerve m_Drivetrain = Swerve.getInstance();
//   public TrajectoryConfiguration() {
//     RobotConfig config;
//     System.out.println("Outside");
//     try {
//       config = RobotConfig.fromGUISettings();
//       System.out.println("TRY CATCH STATEMENT");
//       AutoBuilder.configure(
//         m_PoseEstimator::getPose, 
//         m_PoseEstimator::setCurrentPose,
//         m_Drivetrain::getCurrentChassisSpeeds,
//         (speeds, feedforwards) -> m_Drivetrain.setRobotBassedOffFieldChassisSpeeds(speeds), 
//         new PPHolonomicDriveController(
//           new PIDConstants(5.0, 0.0, 0.0), 
//           new PIDConstants(5.0, 0.0, 0.0)
//         ),
//         config,
//         () -> {
//           var alliance = DriverStation.getAlliance();
//           if (alliance.isPresent()) {
//             return alliance.get() == DriverStation.Alliance.Red;
//           }
//           return false;
//         },
//         this
//       );
//     } catch (Exception e) {
//       System.out.println("-----------------------------");
//       e.printStackTrace();
//     }
//   }
//   public static TrajectoryConfiguration getInstance() {
//     System.out.println("RUN");
//     if (trajectoryConfig == null) {
//       trajectoryConfig = new TrajectoryConfiguration();
//     }
//     return trajectoryConfig;
//   }
//   @Override
//   public void periodic() {
//     SmartDashboard.putString("Current Pose", String.format("(%.6f, %.6f)", m_PoseEstimator.getPose().getX(), m_PoseEstimator.getPose().getY()));
//   }
// }
