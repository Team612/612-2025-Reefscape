// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.*;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.RelativeEncoder;

// import com.studica.frc.AHRS;
// import com.studica.frc.AHRS.NavXComType;

public class Mecanum extends SubsystemBase {
  private static Mecanum mechInstance;
  private static MecanumDrive mech;
  private final double DEADZONE = 0.1;

  public Pigeon2 gyro;
  // private AHRS navx;
  private final SparkMax spark_fl;
  private final SparkMax spark_fr;
  private final SparkMax spark_bl;
  private final SparkMax spark_br;


  /** Creates a new Swerve. */
  public Mecanum() {

    spark_fl = new SparkMax(Constants.DrivetrainConstants.SPARK_FL,MotorType.kBrushless);
    spark_fr = new SparkMax(Constants.DrivetrainConstants.SPARK_FR,MotorType.kBrushless);
    spark_bl = new SparkMax(Constants.DrivetrainConstants.SPARK_BL,MotorType.kBrushless);
    spark_br = new SparkMax(Constants.DrivetrainConstants.SPARK_BR,MotorType.kBrushless);

    SparkMaxConfig sp = new SparkMaxConfig();
    spark_fr.configure(sp.inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    spark_br.configure(sp.inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    spark_fl.configure(sp.inverted(false), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    spark_bl.configure(sp.inverted(false), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    gyro = new Pigeon2(Constants.pigeonID);
    gyro.getConfigurator().apply(new Pigeon2Configuration());
    gyro.setYaw(0);
    gyro.reset();
    mech = new MecanumDrive(spark_fl, spark_bl, spark_fr, spark_br);
    //CHECK THIS LATER!!
    // navx = new AHRS(NavXComType.kMXP_SPI);
    // navx.setAngleAdjustment(Constants.navxAngleOffset);
  }

    public void driveMecanum(double fl, double bl, double fr, double br){
      spark_fl.set(fl);
      spark_bl.set(bl);
      spark_fr.set(fr);
      spark_br.set(br);
    }


  public Rotation2d getPigeonAngle(){
    return gyro.getRotation2d();
    // used to be navx.getRotation2d();
  }

  public MecanumDriveWheelPositions getMecanumDriveWheelPositions(){
    return new MecanumDriveWheelPositions(
        spark_fl.getEncoder().getPosition(), 
        spark_fr.getEncoder().getPosition(), 
        spark_bl.getEncoder().getPosition(),
        spark_br.getEncoder().getPosition()
      );
  }

  public void zeroGyro() {
    gyro.reset();
  }


  // public void resetAlignment() {
  //   for(SwerveModule mod : modules) {
  //     mod.resetToAbsolute();
  //   }
  // }

  public static Mecanum getInstance(){
    if (mechInstance == null){
      mechInstance = new Mecanum();
    }
    return mechInstance;
  }


  
  @Override
  public void periodic() {

  }


  public void FieldOrientedDrive(double x, double y, double zRotation){
    if(Math.abs(x) < DEADZONE) x = 0;
    if(Math.abs(y) < DEADZONE) y = 0;
    if(Math.abs(zRotation) < DEADZONE) zRotation = 0;
    mech.driveCartesian(x, y, zRotation, getPigeonAngle());
  }
  

  public void RobotOrientedDrive(double y, double x, double zRot){
    if(Math.abs(x) < DEADZONE) x = 0;
    if(Math.abs(y) < DEADZONE) y = 0;
    if(Math.abs(zRot) < DEADZONE) zRot = 0;
    mech.driveCartesian(y, x, zRot);
  }
}