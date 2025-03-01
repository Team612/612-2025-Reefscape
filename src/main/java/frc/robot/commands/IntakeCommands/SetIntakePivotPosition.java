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
  private Intake m_intake;
  private Payload m_payload;
  private double position;
  /** Creates a new SetElevatorPosition. */
  public SetIntakePivotPosition(Intake i, Payload pay, double p) {
    m_intake = i;
    m_payload = pay;
    position = p;
    addRequirements(m_intake);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.setPivotSpeed(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // System.out.println(position);
    m_intake.setPivotPosition(position);
    // m_payload.setMotorSpeed(0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("True");
    m_intake.setPivotSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (position >= Constants.IntakeConstants.maxPivotL1Angle && (m_payload.getPosition()) <= Constants.ElevatorConstants.CoralStationPosition - Constants.ElevatorConstants.elevatorThreshold){
      System.out.println("Cannot pivot back, Elevator is too low!");
      return true;
    }
    // System.out.println(Math.abs(m_payload.getPosition() - position));
    System.out.println("Set Intake: " + Math.abs(m_payload.getPosition() - position));
    return (Math.abs(m_intake.getPivotPosition() - position) <= Constants.IntakeConstants.intakeThreshold);
    // if (m_payload.getPosition() < position){
    //   return (m_payload.getPosition() - position) >= 0;
    // }
    // else {
    //   return (m_payload.getPosition() - position) <= 0;
    // }
  }
}
