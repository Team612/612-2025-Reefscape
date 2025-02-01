// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LIDAR;
import frc.robot.commands.LIDARCommand;
public class RobotContainer {
  public RobotContainer() {
    public LIDAR lidara = new LIDAR();
    configureBindings();
  }

  private void configureBindings() {
    LIDAR.setDefaultCommand(lidara);
  }

  public Command getAutonomousCommand() {

  }
}