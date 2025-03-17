// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Payload;
import frc.robot.subsystems.Intake;
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetElevatorPosition extends Command {
  private Payload m_payload;

  private Intake m_intake;
  private double position;
  /** Creates a new SetElevatorPosition. */
  public SetElevatorPosition(Payload pay,Intake i, double p) {
    m_intake = i;
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
    // System.out.println(position);
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
    if (position <= Constants.ElevatorConstants.CoralStationPosition - Constants.ElevatorConstants.elevatorThreshold && m_intake.getPivotPosition() >= Constants.IntakeConstants.maxPivotL1Angle){
      System.out.println("Cannot move elevator, pivot too far back");
      return true;
    }
    System.out.println("Set Elevator: " + Math.abs(m_payload.getPosition() - position));
    //System.out.println(Math.abs(m_payload.getPosition() - position));

    if (Math.abs(m_payload.getPosition() - position) <= Constants.ElevatorConstants.elevatorThreshold && m_payload.magValue() <= Constants.ElevatorConstants.magReached) {
      return true;
    }


    return (Math.abs(m_payload.getPosition() - position) <= Constants.ElevatorConstants.elevatorThreshold);

    // if (m_payload.getPosition() < position){
    //   return (m_payload.getPosition() - position) >= 0;
    // }
    // else {
    //   return (m_payload.getPosition() - position) <= 0;
    // }
  }
}
