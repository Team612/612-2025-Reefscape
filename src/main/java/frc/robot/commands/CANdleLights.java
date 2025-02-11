package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Controls.ControlMap;
import frc.robot.subsystems.CANdleSubsystem;
import frc.robot.subsystems.Mecanum;
import java.util.function.DoubleSupplier;

public class CANdleLights extends Command {
  private CANdleSubsystem m_candle;

  public CANdleLights(CANdleSubsystem candle) {
     m_candle = candle;
  }

  @Override
  public void execute() {
    m_candle.setColor(255, 255, 255);
  }
}