// // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LIDAR;

public class LIDARCommand extends CommandBase {
  /** Creates a new DefaultDrive. */
  
  LIDAR lidar;
  public LIDARCommand(LIDAR lidar) {
    this.lidar = lidar;
    addRequirements(lidar);
  }

  public void execute() {
    System.out.println(lidar.getDistance());
  }
}