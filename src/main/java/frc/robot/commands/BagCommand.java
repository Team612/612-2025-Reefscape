// // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Bag;

public class BagCommand extends Command {
  /** Creates a new DefaultDrive. */
  
  Bag m_s;
  public BagCommand(Bag s) {
    this.m_s = s;
    addRequirements(m_s);
  }

  public void execute() {
    System.out.println(m_s.getDistance());
  }
}