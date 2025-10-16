package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Bag;

public class DriverStationBag extends Command {
  private Bag m_bag;
  private double speed;

  public DriverStationBag(Bag m_bag, double speed) {
    this.m_bag = m_bag;
    this.speed = speed;
    addRequirements(m_bag);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_bag.setBag(speed);
  }

  @Override
  public void end(boolean interrupted) {
    m_bag.setBag(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
