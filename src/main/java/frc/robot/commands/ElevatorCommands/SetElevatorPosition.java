// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Payload;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetElevatorPosition extends Command {
  private Payload m_payload;
  private double position;
  /** Creates a new SetElevatorPosition. */
  public SetElevatorPosition(Payload pay,double p) {
    m_payload = pay;
    position = p;
    addRequirements(m_payload);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_payload.setMotorSpeed(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_payload.setPosition(position);
    // m_payload.setMotorSpeed(0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("True");
    m_payload.setMotorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(m_payload.getPosition() - position) <= Constants.ElevatorConstants.elevatorThreshold);
    // if (m_payload.getPosition() < position){
    //   return (m_payload.getPosition() - position) >= 0;
    // }
    // else {
    //   return (m_payload.getPosition() - position) <= 0;
    // }
  }
}
