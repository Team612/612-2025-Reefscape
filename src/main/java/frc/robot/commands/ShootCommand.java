// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Motor;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootCommand extends Command {
  /** Creates a new ShootCommand. */
  private final Motor m_motor;
  private boolean ready;
  private final long finish;
  private long time;
  public ShootCommand(Motor motor, boolean atLevel, int finish) {
    m_motor = motor;
    ready = atLevel;
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
    SmartDashboard.putNumber("Shoot Velocity", m_motor.getVelocity());
    SmartDashboard.putNumber("Shoot position", m_motor.getPosition());
    if (ready && time == 0) {
      time = System.currentTimeMillis();
      m_motor.setVelocity(1);
    }
    if (System.currentTimeMillis() - time >= finish) {
      m_motor.setVelocity(0);
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
