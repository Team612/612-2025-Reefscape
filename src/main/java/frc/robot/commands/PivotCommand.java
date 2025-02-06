// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PivotCommand extends Command {
  /** Creates a new PivotCommand. */
  private final Pivot m_pivot;
  public PivotCommand() {
    this.m_pivot = new Pivot(new SparkMax(0, MotorType.kBrushless));
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_pivot.setForwardLimitSwitch(false);
    m_pivot.setReverseLimitSwitch(false);
    if (m_pivot.getForwardLimitSwitchPressed()) {
      m_pivot.setVelocity(1);
    }
    if (m_pivot.getReverseLimitSwitchPressed()) {
      m_pivot.setVelocity(0);
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
