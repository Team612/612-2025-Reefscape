// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class Bag extends SubsystemBase {
  /** Creates a new LIDAR. */
  // AnalogInput analog;
  private SparkMax bagMotor;
  Encoder myEncoder = new Encoder(1,2);


  public Bag() {
    bagMotor = new SparkMax(Constants.IntakeConstants.bagID, MotorType.kBrushed);
    bagMotor.configure(MotorConfigs.spark_bag_configs, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);  
  }


  public void setBags(double speed){
    bagMotor.set(speed);
  }

  public SparkMax getBags(){
    return bagMotor;
  }

  public double getBagSpeed(){
    return bagMotor.get();
  }

  public static Bag getInstance(){
    if (instance == null){
      instance = new Bag();
    }
    return instance;
  }

  // public int getDistance() {
    // return bagMoto;
  // }
}
