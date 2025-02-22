// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Motor;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PivotCommand extends Command {
  /** Creates a new motorCommand. */
  private final Motor m_motor;
  private final int level;
  public PivotCommand(Motor motor, int level) { // 0 = L1, 1 = L2, 2 = L3, 4 = coral station
    m_motor = motor;
    this.level = level;
    addRequirements(motor);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double level = this.level == 0 ? Constants.L1 : this.level == 1 ? Constants.L2 : this.level == 3 ? Constants.L3 : Constants.L4;
    if (m_motor.getPosition() > Math.abs(level - 0.1)) {
      m_motor.setVelocity(1);
    } else if (m_motor.getPosition() < Math.abs(level - 0.1)) {
      m_motor.setVelocity(-1);
    } else {
      m_motor.setVelocity(0);
      this.end(true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
