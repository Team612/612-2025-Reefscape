package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Controls.ControlMap;
import frc.robot.subsystems.Payload;
import java.util.function.DoubleSupplier;

public class Reset extends Command {
  private Payload m_pay;

  public Reset(Payload pay) {
     m_pay = pay;
  }

  @Override
  public void execute() {
    m_pay.resetCount();
  }
}