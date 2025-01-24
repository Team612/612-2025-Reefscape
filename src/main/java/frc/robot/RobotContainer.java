// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.FieldRelativeDrive;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Mecanum;

public class RobotContainer {
  private final Mecanum m_drivetrain;
  private final CommandXboxController driverControls;
  private final DefaultDrive m_defaultDrive;
  private final FieldRelativeDrive m_FieldRelativeDrive;
  private final PoseEstimator Pose;
  public RobotContainer() {
    m_drivetrain = Mecanum.getInstance();
    Pose = PoseEstimator.getPoseEstimatorInstance();

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
    m_drivetrain.setDefaultCommand(m_FieldRelativeDrive);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
