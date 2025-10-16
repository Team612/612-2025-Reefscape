package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.BagConstants;
import frc.robot.subsystems.Bag;

public class AutoBagOut extends Command {
  private Bag m_bag;
  private int timer = 0;

  public AutoBagOut(Bag m_bag) {
    this.m_bag = m_bag;
    addRequirements(m_bag);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_bag.setBag(BagConstants.autoBagSpeed);
    timer++;
  }

  @Override
  public void end(boolean interrupted) {
    m_bag.setBag(0);
  }

  @Override
  public boolean isFinished() {
    return timer >= BagConstants.autoBagTime;
  }
}