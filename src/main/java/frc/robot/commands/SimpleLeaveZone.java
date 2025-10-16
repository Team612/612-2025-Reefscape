package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Swerve;

public class SimpleLeaveZone extends Command {

  private Swerve m_swerve;
  private int timer = 0;

  public SimpleLeaveZone(Swerve m_swerve) {
    this.m_swerve = m_swerve;
    addRequirements(m_swerve);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_swerve.drive(new ChassisSpeeds(DriveConstants.simpleLeaveZoneSpeed,0,0));
    timer++;
  }

  @Override
  public void end(boolean interrupted) {
    m_swerve.drive(new ChassisSpeeds());
  }

  @Override
  public boolean isFinished() {
    return (timer >= DriveConstants.simpleLeaveZoneTime);
  }
}
