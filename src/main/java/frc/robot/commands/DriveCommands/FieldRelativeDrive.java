// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Mecanum;
import frc.robot.util.ControlMap;

public class FieldRelativeDrive extends Command {
  /** Creates a new DefaultDrive. */
  
  Mecanum m_drivetrain;
  Constants.DrivetrainConstants m_slowmo;
  public FieldRelativeDrive(Mecanum drivetrain) {
    m_drivetrain = drivetrain;
    m_drivetrain.driveMecanum(0, 0, 0, 0);
    addRequirements(m_drivetrain);
  }

  public void execute() {
    m_drivetrain.FieldOrientedDrive(-1*ControlMap.driver_joystick.getRawAxis(1), -1*ControlMap.driver_joystick.getRawAxis(0), ControlMap.driver_joystick.getRawAxis(4));

  }
}