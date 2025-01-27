package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Mecanum;
import java.util.function.DoubleSupplier;

public class DefaultDrive extends Command {
  Mecanum m_mecanum;
  DoubleSupplier translation;
  DoubleSupplier strafe;
  DoubleSupplier rotation;

  public DefaultDrive(Mecanum m_drive, DoubleSupplier translation, DoubleSupplier strafe, DoubleSupplier rotation){
    m_mecanum = m_drive;
    addRequirements(m_mecanum);
    translation = this.translation;
    strafe = this.strafe;
    rotation = this.rotation;
  }

  @Override
  public void execute() {
     m_mecanum.RobotOrientedDrive(-translation.getAsDouble(), strafe.getAsDouble(), rotation.getAsDouble());
  }
}