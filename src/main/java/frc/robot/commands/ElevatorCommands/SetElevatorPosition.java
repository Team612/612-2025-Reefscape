// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Payload;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.Timer;

public class SetElevatorPosition extends Command {
  private Payload m_payload;
  private Intake m_intake;
  private double position;
  private Timer m_timer;
  private int ticks = 0;
  private PIDController elevatorPID = new PIDController(Constants.ElevatorConstants.kP, 0, 0);

  /** Creates a new SetElevatorPosition. */
  public SetElevatorPosition(Payload pay,Intake i, double p) {
    m_intake = i;
    m_payload = pay;
    position = p;
    m_timer = new Timer();
    addRequirements(m_payload);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.start();
    m_payload.setMotorSpeed(0);
    m_payload.resetPIDController();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  //   double elevatorSpeed = -elevatorPID.calculate(m_payload.getPosition(), position);

  // if (elevatorSpeed > Constants.ElevatorConstants.maxTrapezoidInput) elevatorSpeed = Constants.ElevatorConstants.maxTrapezoidInput;
  //   if (elevatorSpeed < -Constants.ElevatorConstants.maxTrapezoidInput) elevatorSpeed = -Constants.ElevatorConstants.maxTrapezoidInput;

  //   m_payload.setMotorSpeed(elevatorSpeed);
    m_payload.setPosition(position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
    m_timer.reset();
    m_payload.setMotorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_timer.get() >= 2.0){
      return true;
    }
    if (position <= Constants.ElevatorConstants.CoralStationPosition - Constants.ElevatorConstants.elevatorThreshold && m_intake.getPivotPosition() >= Constants.IntakeConstants.maxPivotL1Angle){
      System.out.println("Cannot move elevator, pivot too far back");
      return true;
    }

    if (Math.abs(m_payload.getPosition() - position) <= Constants.ElevatorConstants.elevatorThreshold && m_payload.magValue() <= Constants.ElevatorConstants.magReached) {
      return true;
    }

    if (Math.abs(m_payload.getPosition() - position) <= Constants.ElevatorConstants.elevatorThreshold)
      ticks++;
    else
      ticks = 0;
    
    if (ticks == 5)
      return true;
    return false;
  }
}
