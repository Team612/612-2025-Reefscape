// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Payload;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PivotIntakeOut extends Command {
  /** Creates a new BagIn. */
  private Intake m_intake;
  private Payload m_payload;
  public PivotIntakeOut(Intake intake, Payload p) {
    m_intake = intake;
    m_payload = p;
    addRequirements(m_intake,m_payload);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.setPivotSpeed(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.setPivotSpeed(Constants.IntakeConstants.pivotspeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setPivotSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_intake.getIntakeLimitStateForward() || m_intake.getPivotPosition() > Constants.IntakeConstants.maxPivotOutAngle;
  }
}
