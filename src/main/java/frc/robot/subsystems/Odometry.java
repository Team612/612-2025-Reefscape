// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.I2C;

public class Odometry extends SubsystemBase {
  private final I2C i2c;
  //private final byte i2cAdress = 0x62;

  // Position tracking (in sensor units)
  private int x = 0;
  private int y = 0;


  public Odometry() {
    i2c = new I2C(I2C.Port.kOnboard, 0x62); //idk if the adress is correct
    if (i2c.addressOnly()) {
      System.out.println("Device address for I2C is incorrect");
    }
  }

  @Override
  public void periodic() {
    int[] motion = readMotion();

    x += motion[0];
    y += motion[1];
  }

  public int[] readMotion() {
    byte[] buffer = new byte[4];
    boolean error = i2c.read(0x8f, 4, buffer);

    if (error) {
      System.out.println("Error reading from Optical Sensor");
      return new int[]{0, 0};
    }

    int dx = ((buffer[1] << 8) | (buffer[0] & 0xFF));
    int dy = ((buffer[3] << 8) | (buffer[2] & 0xFF));

    // Optional: handle signed values if needed
    dx = (short) dx;
    dy = (short) dy;

    return new int[]{dx, dy};
  }

  public int getX() {
    return x;
  }

  public int getY() {
    return y;
  }

  public void resetPos() {
    x = 0;
    y = 0;
  }
}
