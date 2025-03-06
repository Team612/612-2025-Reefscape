// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.MotorConfigs;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.CANdle;
public class Leds extends SubsystemBase {
  private CANdle candle;
  private static Leds instance;
  /** Creates a new Leds. */
  public Leds() {
    candle = new CANdle(Constants.LedConstants.candleID);
    setColor(255, 255, 255);
    candle.configAllSettings(MotorConfigs.LED_configs);

  }

  
  public void setColor(int red, int green, int blue){
    candle.animate(null);
    candle.setLEDs(red, green, blue);
  }

  public void waterAnimation(){
    ColorFlowAnimation colorFlow = new ColorFlowAnimation(0, 0, 255);
    candle.animate(colorFlow);
  }

  public static Leds getInstance(){
    if (instance == null){
      instance = new Leds();
    }
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
