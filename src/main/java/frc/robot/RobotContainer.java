// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SwivelElevator;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.FieldRelativeDrive;
import frc.robot.subsystems.Payload;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
  private final Swerve m_drivetrain;
  private final Payload m_pay;
  private final CommandXboxController driverControls;
  private final DefaultDrive m_defaultDrive;
  private final FieldRelativeDrive m_FieldRelativeDrive;
  private final Command m_movePay;

  public RobotContainer() {
    m_drivetrain = Swerve.getInstance();
    m_pay = Payload.getInstance();

    driverControls = new CommandXboxController(Constants.driverPort);
    m_defaultDrive = new DefaultDrive( m_drivetrain,
            () -> -driverControls.getLeftY(),
            () -> -driverControls.getLeftX(),
            () -> -driverControls.getRightX());

    m_FieldRelativeDrive = new FieldRelativeDrive( m_drivetrain,
            () -> -driverControls.getLeftY(),
            () -> -driverControls.getLeftX(),
            () -> -driverControls.getRightX());

    m_movePay = new SwivelElevator(m_pay);
    configureBindings();
  }

  private void configureBindings() {
    driverControls.b().toggleOnTrue(m_defaultDrive);
    driverControls.a().toggleOnTrue(m_movePay);
    m_drivetrain.setDefaultCommand(m_FieldRelativeDrive);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}