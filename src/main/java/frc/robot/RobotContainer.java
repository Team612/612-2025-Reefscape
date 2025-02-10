// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.FieldRelativeDrive;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;


public class RobotContainer {
  private final Swerve m_drivetrain;
  private final CommandXboxController driverControls;
  private final DefaultDrive m_defaultDrive;
  private final FieldRelativeDrive m_FieldRelativeDrive;
  private final AprilTagFieldLayout m_fieldLayout;
  public RobotContainer() {
    m_fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    m_drivetrain = Swerve.getInstance();
    driverControls = new CommandXboxController(Constants.driverPort);
    m_defaultDrive = new DefaultDrive(m_drivetrain);

    m_FieldRelativeDrive = new FieldRelativeDrive( m_drivetrain,
            () -> -driverControls.getLeftY(),
            () -> -driverControls.getLeftX(),
            () -> -driverControls.getRightX());

    configureBindings();
  }

  private void configureBindings() {
    // driverControls.b().toggleOnTrue(m_defaultDrive);
    m_drivetrain.setDefaultCommand(m_defaultDrive);
    driverControls.leftBumper().onTrue(new InstantCommand(() -> m_drivetrain.zeroGyro()));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
