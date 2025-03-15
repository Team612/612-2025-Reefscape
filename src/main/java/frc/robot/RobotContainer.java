// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ArcadeDriveCommand;
import frc.robot.commands.resetGyro;
import frc.robot.subsystems.mySwerveSubsystem;

public class RobotContainer {
  public RobotContainer() {
    configureBindings();
  }

  mySwerveSubsystem m_swervesubsystem = new mySwerveSubsystem();
  CommandXboxController controller = new CommandXboxController(Constants.XboxPortNumber);

  private void configureBindings() {
    m_swervesubsystem.setDefaultCommand(new ArcadeDriveCommand(() -> -controller.getLeftY()*Constants.xMultiple, () -> -controller.getLeftX()*Constants.yMultiple, () -> -controller.getRightX()*Constants.zMultiple, m_swervesubsystem));
    controller.a().onTrue(new resetGyro(m_swervesubsystem));
  }

  public Command getAutonomousCommand() {
    try{
        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromPathFile("Go Forward 1 Meter");

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path);
    } catch (Exception e) {
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        return Commands.none();
    }
  }
}
