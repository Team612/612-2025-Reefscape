// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.DriverCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Mecanum;
import frc.robot.subsystems.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PIDAlign extends Command {
  private final Mecanum m_drivetrain;
  private final Vision m_vision;
  private final double m_side;

  private PIDController m_xTranslationController = new PIDController(1, 0, 0);
  private PIDController m_yTranslationController = new PIDController(1, 0, 0);
  private PIDController m_rotationController = new PIDController(1, 0, 0);
  /** Creates a new AutoAlign. */
  public PIDAlign(Mecanum drivetrain, Vision vision, int side) {
    m_drivetrain = drivetrain;
    m_vision = vision;
    m_side = side;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xApplied = -m_xTranslationController.calculate(m_vision.getFrontRelativeTagPose().getX(), 0.5);
		double yApplied = -m_yTranslationController.calculate(m_vision.getFrontRelativeTagPose().getY(), m_side * 0.2);
    //rotation may need to be negated here
    double rotationApplied = -m_rotationController.calculate(m_vision.getFrontRelativeTagPose().getRotation().getRadians(), 0);

    m_drivetrain.AutoDrive(new ChassisSpeeds(xApplied, yApplied, rotationApplied));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.AutoDrive(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_vision.getFrontRelativeTagPose().getX() - 0.5) < 0.1 && Math.abs(m_vision.getFrontRelativeTagPose().getY() - m_side * 0.2) < 0.1 && m_vision.getFrontRelativeTagPose().getRotation().getDegrees() < 1;
  }
}
