package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Controls.ControlMap;
import frc.robot.subsystems.Swerve;
import java.util.function.DoubleSupplier;

public class DefaultDrive extends Command {
  private Swerve m_drivetrain;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

  public DefaultDrive(Swerve drive) {
    this.m_drivetrain = drive;
    addRequirements(drive);
  }

  @Override
  public void execute() {
    /* Get Values, Deadband*/
    double translationVal =
        translationLimiter.calculate(
            MathUtil.applyDeadband(ControlMap.gunner_buttons.getRawAxis(0), Constants.stickDeadband));
    double strafeVal =
        strafeLimiter.calculate(
            MathUtil.applyDeadband(ControlMap.gunner_buttons.getRawAxis(1), Constants.stickDeadband));

    double rotationVal = 0;
    if (ControlMap.gunner_buttons.getRawButton(2)){
      rotationVal = 1;
    }
    if (ControlMap.gunner_buttons.getRawButton(8)){
      rotationVal = -1;
    }
    // double rotationVal =
    //     rotationLimiter.calculate(
    //         MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband));

    /* Drive */
 
    System.out.println(rotationVal * Constants.maxSpeed);
    m_drivetrain.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.maxSpeed),
        rotationVal * Constants.maxAngularVelocity,
        true, true);
  }
}