package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PayloadConstants;
import frc.robot.subsystems.Payload;

public class DriverStationArcadeGun extends Command {
  private Payload m_payload;
  private Joystick gunner_controls;

  public DriverStationArcadeGun(Payload m_payload, Joystick gunner_controls) {
    this.m_payload = m_payload;
    this.gunner_controls = gunner_controls;
    addRequirements(m_payload);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (gunner_controls.getRawAxis(0) == -1)
      m_payload.setElevator(PayloadConstants.driveStationElevatorDownSpeed);
    else if (gunner_controls.getRawAxis(0) == 1)
      m_payload.setElevator(-PayloadConstants.driveStationElevatorUpSpeed);
    else
      m_payload.setElevator(0);

    if (gunner_controls.getRawAxis(1) == -1)
      m_payload.setIntake(-PayloadConstants.driveStationPivotSpeed);
    else if (gunner_controls.getRawAxis(1) == 1)
      m_payload.setIntake(PayloadConstants.driveStationPivotSpeed);
    else
      m_payload.setIntake(0);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
