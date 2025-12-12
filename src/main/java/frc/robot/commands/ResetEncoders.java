package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Swerve;

public class ResetEncoders extends InstantCommand {
  Swerve m_swerve;

  public ResetEncoders(Swerve m_swerve) {
    this.m_swerve = m_swerve;
    addRequirements(m_swerve);
  }

  @Override
  public void initialize() {
    m_swerve.setPose(new Pose2d());
  }
}
