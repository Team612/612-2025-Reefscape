package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Bag;

public class ManualBag extends Command {
  private Bag m_bag;
  private CommandXboxController controller;

  public ManualBag(Bag m_bag, CommandXboxController controller) {
    this.m_bag = m_bag;
    this.controller = controller;
    addRequirements(m_bag);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_bag.setBag(controller.getRightTriggerAxis()-controller.getLeftTriggerAxis());
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
