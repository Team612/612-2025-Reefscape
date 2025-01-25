// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.FieldRelativeDrive;
import frc.robot.commands.RunOnTheFly;
import frc.robot.commands.TrajectoryCreation;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.TrajectoryConfiguration;
import frc.robot.subsystems.Vision;

public class RobotContainer {
  private final Swerve m_drivetrain;
  private final TrajectoryCreation m_traj;
  private final Vision m_vision;
  private final RunOnTheFly runOnTheFly;
  private final CommandXboxController driverControls;
  private final DefaultDrive m_defaultDrive;
  private final FieldRelativeDrive m_FieldRelativeDrive;
  private final PoseEstimator m_poseEstimator;
  private final TrajectoryConfiguration m_trajConfig;
  public RobotContainer() {

    m_drivetrain = Swerve.getInstance();
    m_poseEstimator = PoseEstimator.getPoseEstimatorInstance();
    m_vision = Vision.getVisionInstance();
    m_traj = new TrajectoryCreation();
    m_trajConfig = TrajectoryConfiguration.getInstance();
    runOnTheFly = new RunOnTheFly(m_drivetrain, m_poseEstimator, m_traj, m_vision, 0);

    driverControls = new CommandXboxController(Constants.driverPort);
    m_defaultDrive = new DefaultDrive( m_drivetrain,
            () -> -driverControls.getLeftY(),
            () -> -driverControls.getLeftX(),
            () -> -driverControls.getRightX());

    m_FieldRelativeDrive = new FieldRelativeDrive( m_drivetrain,
            () -> -driverControls.getLeftY(),
            () -> -driverControls.getLeftX(),
            () -> -driverControls.getRightX());

    configureBindings();
  }

  private void configureBindings() {
    driverControls.b().toggleOnTrue(m_defaultDrive);
    driverControls.a().onTrue(runOnTheFly);
    m_drivetrain.setDefaultCommand(m_defaultDrive);
  }
  
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
