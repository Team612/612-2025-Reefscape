// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.control.ControlMap;
import frc.robot.subsystems.Mecanum;

public class DefaultDrive extends Command {
  /** Creates a new DefaultDrive. */
  
  Mecanum m_drivetrain;
  Constants.DrivetrainConstants m_slowmo;
  public DefaultDrive(Mecanum drivetrain) {
    m_drivetrain = drivetrain;
    m_drivetrain.driveMecanum(0, 0, 0, 0);
  }

  public void execute() {
    m_drivetrain.RobotOrientedDrive(-ControlMap.driver_joystick.getRawAxis(1), ControlMap.driver_joystick.getRawAxis(0), ControlMap.driver_joystick.getRawAxis(4));
  }
}