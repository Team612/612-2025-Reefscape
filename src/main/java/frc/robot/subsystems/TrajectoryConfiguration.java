// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;


import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TrajectoryConfiguration extends SubsystemBase {
  private static TrajectoryConfiguration trajectoryConfig;

  private PoseEstimator m_PoseEstimator = PoseEstimator.getPoseEstimatorInstance();
  private Mecanum m_Drivetrain = Mecanum.getInstance();
  /** Creates a new TrajectoryConfiguration. */
  public TrajectoryConfiguration() {
    //   AutoBuilder.configure(
    //         this::getPose, // Robot pose supplier
    //         this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
    //         this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
    //         (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
    //         new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
    //                 new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
    //                 new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
    //         ),
    //         config, // The robot configuration
    //         () -> {
    //           // Boolean supplier that controls when the path will be mirrored for the red alliance
    //           // This will flip the path being followed to the red side of the field.
    //           // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

    //           var alliance = DriverStation.getAlliance();
    //           if (alliance.isPresent()) {
    //             return alliance.get() == DriverStation.Alliance.Red;
    //           }
    //           return false;
    //         },
    //         this // Reference to this subsystem to set requirements
    // );
    RobotConfig config;
    System.out.println("Outside");
    try{
      config = RobotConfig.fromGUISettings();
      System.out.println("TRY CATCH STATEMENT");
      AutoBuilder.configure(
            m_PoseEstimator::getPose, // Robot pose supplier
            m_PoseEstimator::setCurrentPose, // Method to reset odometry (will be called if your auto has a starting pose)
            m_Drivetrain::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> m_Drivetrain.RobotOrientedDrive(speeds.vyMetersPerSecond,speeds.vxMetersPerSecond,speeds.omegaRadiansPerSecond), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(0.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(0.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
      );
    } catch (Exception e) {
      
      // Handle exception as needed
      System.out.println("-----------------------------------");
      e.printStackTrace();
    }
  }

  public static TrajectoryConfiguration getInstance(){
    System.out.println("RUN");
    if (trajectoryConfig == null){
        trajectoryConfig = new TrajectoryConfiguration();
    }
    return trajectoryConfig;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("Current Pose", m_PoseEstimator.getPose().getX() + ", " + m_PoseEstimator.getPose().getY());
  }
}