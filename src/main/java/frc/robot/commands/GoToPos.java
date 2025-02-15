package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Controls.ControlMap;
import frc.robot.subsystems.Payload;
import java.util.function.DoubleSupplier;

public class GoToPos extends Command {
  private Payload m_pay;


  public GoToPos(Payload pay) {
     m_pay = pay;
     while (true){
        m_pay.setMotorSpeed(1);
        if (m_pay.Pos() == Constants.encB){
          m_pay.freezeMotors();
            break;
        }
     }
  }
}