package frc.robot.commands.IntakeCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Bag;

public class AutoIntakeCoral extends Command {

  private Intake m_intake;
  private Bag m_bag;
  private double ticks;
  public AutoIntakeCoral(Intake intake, Bag bag) {
    m_intake = intake;
    m_bag = bag;
    addRequirements(intake, bag);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_bag.setBags(Constants.IntakeConstants.pivotspeed);
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.setPivotSpeed(0);
    m_bag.setBags(0);
  }

  @Override
  public boolean isFinished() {
    if (ticks == 10)
      return true;
    return false;
  }
}