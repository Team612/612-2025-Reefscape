package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Controls.ControlMap;
import frc.robot.subsystems.Payload;
import java.util.function.DoubleSupplier;

public class IncrementPay extends Command {
  public IncrementPay(double num) {
     Constants.payspeed += num;
  }
}