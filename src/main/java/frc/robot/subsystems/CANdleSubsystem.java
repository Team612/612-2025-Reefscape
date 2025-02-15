// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// import com.studica.frc.AHRS;
// import com.studica.frc.AHRS.NavXComType;

public class CANdleSubsystem extends SubsystemBase {
  private static CANdleSubsystem canInstance;
  private final CANdle m_can;

  public CANdleSubsystem() {
    m_can = new CANdle(Constants.canId);
    m_can.clearStickyFaults();
    m_can.configBrightnessScalar(Constants.brightness);
  }

  public void setColor(int red, int green, int blue){
      m_can.setLEDs(red, green, blue);
  }

  public static CANdleSubsystem getInstance(){
    if (canInstance == null){
      canInstance = new CANdleSubsystem();
    }
    return canInstance;
  }


  @Override
  public void periodic() {

  }
}