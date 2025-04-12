package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Swerve;

public class resetGyro extends InstantCommand {
  private Swerve m_swervesubsystem;

  public resetGyro(Swerve subsystem) {
    m_swervesubsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swervesubsystem.resetGyro();
    m_swervesubsystem.resetEncoders();
  }
}
