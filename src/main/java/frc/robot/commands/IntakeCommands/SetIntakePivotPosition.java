// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Payload;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetIntakePivotPosition extends Command {
  /** Creates a new SetPivotPosition. */
  private Intake m_intake;
  private Payload m_payload;
  private double position;
  public SetIntakePivotPosition(Intake intake, Payload pay, double p) {
    m_intake = intake;
    m_payload = pay;
    position = p;
    addRequirements(m_intake, m_payload);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  m_intake.setPivotSpeed(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.setPivotPosition(position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setPivotSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //software limits
    if (m_payload.getPosition() < Constants.ElevatorConstants.CoralStationPosition
    && position > Constants.IntakeConstants.maxPivotInAngleL1){
      return true;
    }

    else {
      if (position > Constants.IntakeConstants.maxPivotOutAngle || position < Constants.IntakeConstants.maxPivotInAngleL2L3){
        return true;
      }
    }

    return (Math.abs(m_intake.getPivotPosition() - position) <= Constants.IntakeConstants.pivotThreshold);
    
    // if (m_intake.getPivotPosition() < position){
    //   return (m_intake.getPivotPosition() - position) >= 0;
    // }
    // else {
    //   return (m_intake.getPivotPosition() - position) <= 0;
    // }
}
}
