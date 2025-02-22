// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Motor;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootCommand extends Command {
  /** Creates a new ShootCommand. */
  private final Motor m_motor;
  private boolean ready;
  private final long finish;
  private long time;
  private int level;
  public ShootCommand(Motor motor, boolean atLevel, int finish, int level) { // 1 = L1, 2 = L2, 3 = L3
    m_motor = motor;
    ready = atLevel;
    this.level = level;
    this.finish = finish;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (ready && time == 0) {
      time = System.currentTimeMillis();
      m_motor.setVelocity(level == 3 ? 0.7 : level == 2 ? 0.5 : 0.275);
    }
    if (System.currentTimeMillis() - time >= finish) {
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
