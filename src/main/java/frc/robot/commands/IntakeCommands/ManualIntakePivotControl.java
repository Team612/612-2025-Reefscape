// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Payload;
import frc.robot.util.ControlMap;

public class ManualIntakePivotControl extends Command {
  private Intake m_intake;
  /** Creates a new ManualElevatorControl. */
  public ManualIntakePivotControl(Intake i) {
    m_intake = i;
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
    m_intake.setPivotSpeed(ControlMap.gunner_controls.getRawAxis(1) * Constants.IntakeConstants.pivotspeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setPivotSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
