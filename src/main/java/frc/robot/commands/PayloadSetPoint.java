package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PayloadConstants;
import frc.robot.subsystems.Payload;

public class PayloadSetPoint extends Command {
  private Payload m_payload;

  private boolean safe;
  private boolean pause = false;
  private int pauseTimer = 25;

  private double elevatorSetPoint;
  private double intakeSetPoint;

  // test out min speed
  private double minSpeed = 0.4;
  private double minDistance = 3;

  public PayloadSetPoint(Payload m_payload, double elevatorSetPoint, double intakeSetPoint) {
    this.m_payload = m_payload;
    this.elevatorSetPoint = elevatorSetPoint;
    this.intakeSetPoint = intakeSetPoint;
    addRequirements(m_payload);
  }

  @Override
  public void initialize() {
    if (m_payload.getIntakePos() < PayloadConstants.CoralStationIntakePosition+30)
      pause = true;
    }

  @Override
  public void execute() {
    if (m_payload.getIntakePos() > PayloadConstants.minSafePivotPosition)
      safe = true;
    else
      safe = false;

    // test out min speed
    // m_payload.setIntake(-m_payload.getIntakePID(intakeSetPoint));
    if ((Math.abs(m_payload.getElevatorPos()-elevatorSetPoint)) < minDistance)
      m_payload.setIntake(-m_payload.getIntakePID(intakeSetPoint));
    else {
      if ((-m_payload.getIntakePID(intakeSetPoint)) > 0)
        m_payload.setIntake(-m_payload.getIntakePID(intakeSetPoint)+minSpeed);
      else
        m_payload.setIntake(-m_payload.getIntakePID(intakeSetPoint)-minSpeed);
    }

    if ((!pause) || (pauseTimer == 0)){
      if (elevatorSetPoint<PayloadConstants.maxSafeHeight)
        m_payload.setElevator(m_payload.getElevatorPID(elevatorSetPoint));
      else if ((elevatorSetPoint>PayloadConstants.maxSafeHeight) && (safe))
        m_payload.setElevator(m_payload.getElevatorPID(elevatorSetPoint));
      else
        m_payload.setElevator(m_payload.getElevatorPID(PayloadConstants.maxSafeHeight));
    }
    else{
      pauseTimer--;
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
