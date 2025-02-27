// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Payload;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorDown extends Command {
  /** Creates a new ElevatorManual. */
  private Payload m_payload;
    public ElevatorDown(Payload payload) {
      m_payload = payload;
    addRequirements(m_payload);
    // Use addRequirements() here to declare subsysstem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_payload.setMotorSpeed(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_payload.setMotorSpeed(-Constants.ElevatorConstants.payloadDownSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_payload.setMotorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_payload.getElevatorMotor().getReverseLimitSwitch().isPressed();
  }
}
