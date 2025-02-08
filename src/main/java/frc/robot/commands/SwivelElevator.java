package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Payload;
import frc.robot.subsystems.Swerve;
import java.util.function.DoubleSupplier;

public class SwivelElevator extends Command {
  private Payload m_pay;

  public SwivelElevator(Payload pay) {
     m_pay = pay;
  }

  @Override
  public void execute() {
    m_pay.elevate();
    m_pay.pivot();
  }
}