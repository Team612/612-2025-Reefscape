// // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Sensor;

public class SensorCommand extends Command {
  /** Creates a new DefaultDrive. */
  
  Sensor m_s;
  public SensorCommand(Sensor s) {
    this.m_s = s;
    addRequirements(m_s);
  }

  public void execute() {
    System.out.println(m_s.getDistance());
  }
}