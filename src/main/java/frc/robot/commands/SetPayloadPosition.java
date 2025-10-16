package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Payload;

public class SetPayloadPosition extends InstantCommand {
  public Payload m_payload;

  private double elevatorPos;
  private double pivotPos;

  public SetPayloadPosition(Payload m_payload, double elevatorPos, double pivotPos) {
    this.m_payload = m_payload;
    this.elevatorPos = elevatorPos;
    this.pivotPos = pivotPos;
    addRequirements(m_payload);
  }

  @Override
  public void initialize() {
    m_payload.setElevatorPos(elevatorPos);
    m_payload.setIntakePos(pivotPos);
  }
}
