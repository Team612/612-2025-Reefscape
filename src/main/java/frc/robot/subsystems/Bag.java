// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.subsystems.Payload;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.MotorConfigs;

public class Bag extends SubsystemBase {
  /** Creates a new Intake. */
  private SparkMax bagMotor;
  private static Bag instance;

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

  @Override
  public void periodic() {
  }
}
