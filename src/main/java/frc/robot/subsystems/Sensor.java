// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;

public class Sensor extends SubsystemBase {
  /** Creates a new LIDAR. */
  AnalogInput analog;

  public Sensor() {
	    analog = new AnalogInput(3);
  }

  public void initLidar() {		
    // nothing to do
  }
  public int getDistance() {
    return analog.getValue();
  }
}
