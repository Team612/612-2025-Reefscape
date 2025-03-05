// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.DriverCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Mecanum;
import frc.robot.subsystems.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SimpleAlign extends Command {
  private final Mecanum m_drivetrain;
  private final Vision m_vision;

  double xApplied = 0;
  double yApplied = 0;
  double rotationApplied = 0;

  /** Creates a new AutoAlign. */
  public SimpleAlign(Mecanum drivetrain, Vision vision) {
    m_drivetrain = drivetrain;
    m_vision = vision;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(m_vision.getFrontRelativeTagPose().getX() - 0.5) < 0.1) {
      if(m_vision.getFrontRelativeTagPose().getX() > 0.5) {
        xApplied = 0.1;
      } else {
        xApplied = -0.1;
      }
    }
    if(Math.abs(m_vision.getFrontRelativeTagPose().getY() - 0.2) < 0.1) {
      if(m_vision.getFrontRelativeTagPose().getY() > 0) {
        yApplied = 0.1;
      } else {
        yApplied = -0.1;
      }
    }
    //rotation may need to be negated here
    if(Math.abs(m_vision.getFrontRelativeTagPose().getRotation().getDegrees()) < 1) {
      if(m_vision.getFrontRelativeTagPose().getRotation().getDegrees() > 0) {
        rotationApplied = 0.1;
      } else {
        rotationApplied = -0.1;
      }
    }

    m_drivetrain.RobotOrientedDrive(xApplied, yApplied, rotationApplied);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.RobotOrientedDrive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
