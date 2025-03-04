// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Payload;
import frc.robot.util.ControlMap;

public class ManualElevatorControl extends Command {
  private Payload m_payload;
  /** Creates a new ManualElevatorControl. */
  public ManualElevatorControl(Payload p) {
    m_payload = p;
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
    if (ControlMap.gunner_controls.getRawAxis(0) == 1){
      m_payload.setMotorSpeed(-ControlMap.gunner_controls.getRawAxis(0) * Constants.ElevatorConstants.payloadDownSpeed);
    }
    else if (ControlMap.gunner_controls.getRawAxis(0) == -1){
    m_payload.setMotorSpeed(-ControlMap.gunner_controls.getRawAxis(0) * Constants.ElevatorConstants.payloadUpSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_payload.setMotorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // System.out.println(ControlMap.gunner_controls.getRawAxis(0));
    return Math.abs(ControlMap.gunner_controls.getRawAxis(0)) <= 0.1;
  }
}
