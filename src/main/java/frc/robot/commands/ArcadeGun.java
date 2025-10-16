package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.PayloadConstants;
import frc.robot.subsystems.Payload;

public class ArcadeGun extends Command {
  private Payload m_payload;
  private CommandXboxController controller;

  public ArcadeGun(Payload m_payload, CommandXboxController controller) {
    this.m_payload = m_payload;
    this.controller = controller;
    addRequirements(m_payload);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double y = controller.getLeftY();
    double x = controller.getRightX();

    if (Math.abs(y) < PayloadConstants.controllingDEADBAND) y = 0;
    if (Math.abs(x) < PayloadConstants.controllingDEADBAND) x = 0;

    m_payload.setElevator(y * PayloadConstants.maxGunnerElevatorControlSpeed);
    m_payload.setIntake(x * PayloadConstants.maxGunnerPivotControlSpeed);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
