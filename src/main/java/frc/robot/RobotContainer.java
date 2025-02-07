// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.PoseEstimator;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.FeedForwardCharacterization.FeedForwardCharacterizationData;
import frc.robot.commands.FieldRelativeDrive;
import frc.robot.control.ControlMap;
import frc.robot.subsystems.Mecanum;
import frc.robot.subsystems.Vision;

public class RobotContainer {
  private final Mecanum m_drivetrain;
  private final DefaultDrive m_defaultDrive;
  private final FieldRelativeDrive m_FieldRelativeDrive;
  private final PoseEstimator m_pose;
  private final Vision m_vision;
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  public RobotContainer() {
    m_drivetrain = Mecanum.getInstance();
    m_defaultDrive = new DefaultDrive(m_drivetrain);
    m_pose = PoseEstimator.getPoseEstimatorInstance();
    m_vision = Vision.getVisionInstance();
    m_FieldRelativeDrive = new FieldRelativeDrive(m_drivetrain);
    configureBindings();
    configureShuffle();
  }

  private void configureBindings() {
    ControlMap.driver_joystick.leftBumper().onTrue(new InstantCommand(() -> m_drivetrain.zeroGyro()));
    m_drivetrain.setDefaultCommand(m_defaultDrive);

    // m_drivetrain.con
    // ControlMap.driver_joystick.
    //     a().whileTrue(m_drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // ControlMap.driver_joystick.
    //     a().whileTrue(m_drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // ControlMap.driver_joystick.
    //     a().whileTrue(m_drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // ControlMap.driver_joystick.
    //     a().whileTrue(m_drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
  }

  public void configureShuffle(){
    m_chooser.addOption("Swerve Characterization", new FeedForwardCharacterization(
              m_drivetrain,
              true,
              new FeedForwardCharacterizationData("drive"),
              m_drivetrain::runCharacterizationVolts,
              m_drivetrain::getCharacterizationVelocity));

    SmartDashboard.putData(m_chooser);
  }
  
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
