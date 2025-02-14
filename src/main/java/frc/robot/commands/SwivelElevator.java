package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Controls.ControlMap;
import frc.robot.subsystems.Payload;
import java.util.function.DoubleSupplier;

public class SwivelElevator extends Command {
  private Payload m_pay;
  private double DEADZONE = 0.1;

  public SwivelElevator(Payload pay) {
     m_pay = pay;
  }

  @Override
  public void execute() {
    if (Math.abs(ControlMap.driver_joystick.getRawAxis(0)) < DEADZONE){
      m_pay.setMotorSpeed(0);
    }
    else{
      m_pay.setMotorSpeed(Constants.payspeed);
    }
  }
}