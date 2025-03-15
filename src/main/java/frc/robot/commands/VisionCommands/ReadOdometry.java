// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.VisionCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.I2C;
import frc.robot.subsystems.Odometry;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ReadOdometry extends Command {
  /** Creates a new ReadOdometry. */
  Odometry m_odometry;
  I2C m_I2C;

  public ReadOdometry(Odometry odometry) {
    m_I2C = new I2C(I2C.Port.kOnboard, 0x62); //port and address could be wrong idk
    m_odometry = odometry;
    addRequirements(m_odometry);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_odometry.resetPos();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("x: " + m_odometry.getX());
    System.out.println("y: " + m_odometry.getY());
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
