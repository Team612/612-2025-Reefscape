package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PayloadConstants;
import frc.robot.subsystems.Payload;

public class Zero extends Command {
  private Payload m_payload;

  public Zero(Payload m_payload) {
    this.m_payload = m_payload;
    addRequirements(m_payload);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (m_payload.getIntakePos() != 0)
      m_payload.setIntake(PayloadConstants.minPivotSpeed);
    else
      m_payload.setElevator(PayloadConstants.elevatorZeroSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    m_payload.setElevator(0);
    m_payload.setIntake(0);
  }

  @Override
  public boolean isFinished() {
    return (m_payload.getElevatorPos() == 0) && (m_payload.getIntakePos() == 0);
  }
}
