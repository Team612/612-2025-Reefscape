// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arms;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmsCommand extends Command {
  /** Creates a new ArmsCommand. */
  private final Arms m_arms;
  public ArmsCommand() {
    this.m_arms = new Arms(new SparkMax(0, MotorType.kBrushed));
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arms.setForwardLimitSwitch(false);
    m_arms.setReverseLimitSwitch(false);
    if (m_arms.getForwardLimitSwitchPressed()) {
      m_arms.setVelocity(1);
    }
    if (m_arms.getReverseLimitSwitchPressed()) {
      m_arms.setVelocity(0);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
