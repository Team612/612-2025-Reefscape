// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Payload;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class setIntakePosition extends Command {
  private Intake m_in;
  private double position;
  /** Creates a new SetElevatorPosition. */
  public setIntakePosition(Intake in,double p) {
    m_in = in;
    position = p;
    addRequirements(m_in);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_in.setPivotSpeed(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_in.setPivotPosition(position);
    // m_payload.setMotorSpeed(0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("True");
    m_in.setPivotSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(m_in.getPivotPosition() - position) <= Constants.IntakeConstants.intakeThreshold);
    // if (m_payload.getPosition() < position){
    //   return (m_payload.getPosition() - position) >= 0;
    // }
    // else {
    //   return (m_payload.getPosition() - position) <= 0;
    // }
  }
}
