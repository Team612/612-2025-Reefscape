// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;

public class Sensor extends SubsystemBase {
  /** Creates a new LIDAR. */
  private static I2C m_i2c;
  public Sensor() {
	  m_i2c = new I2C(I2C.Port.kMXP,0x52);
	  initLidar();     	          	 
  }
  public void initLidar() {		
    // nothing to do
  }
  public int getDistance() {
    byte[] buffer = new byte[2];
    m_i2c.write(0x00, 0x04);
    Timer.delay(0.04);

    if (!m_i2c.read(0x8f, 2, buffer)) {
      System.out.println("Error reading Color data");
      return -1;
    } 	
    return ((buffer[0] & 0xFF) << 8) | (buffer[1] & 0xFF);
  }
  @Override
  public void setDefaultCommand(Command defaultCommand) {
      // TODO Auto-generated method stub
      super.setDefaultCommand(defaultCommand);
  }
}