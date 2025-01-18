// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.*;
import frc.robot.util.SparkConfigs;
import com.revrobotics.spark.*;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.SwerveModule;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.RelativeEncoder;

// import com.studica.frc.AHRS;
// import com.studica.frc.AHRS.NavXComType;

public class Mechanum extends SubsystemBase {
  private static Mechanum swerveInstance;
  private SwerveModule[] modules;
  private SparkConfigs swerveModuleConfigs;
  private SwerveDriveOdometry odometry;
  public Pigeon2 gyro;
  // private AHRS navx;
  private final Spark spark_fl;
  private final Spark spark_fr;
  private final Spark spark_bl;
  private final Spark spark_br;


  /** Creates a new Swerve. */
  public Mechanum() {

    spark_fl = new Spark(Constants.DrivetrainConstants.SPARK_FL);
    spark_fr = new Spark(Constants.DrivetrainConstants.SPARK_FR);
    spark_bl = new Spark(Constants.DrivetrainConstants.SPARK_BL);
    spark_br = new Spark(Constants.DrivetrainConstants.SPARK_BR);
    
    spark_fr.setInverted(true);
    spark_br.setInverted(true);
    spark_fl.setInverted(false);
    spark_bl.setInverted(false);
    
    spark_fl.setIdleMode(IdleMode.kBrake);
    spark_fr.setIdleMode(IdleMode.kBrake);
    spark_bl.setIdleMode(IdleMode.kBrake);
    spark_br.setIdleMode(IdleMode.kBrake);


    spark_fr.getEncoder().setPositionConversionFactor(Constants.DrivetrainConstants.kEncoderDistancePerPulse);
    spark_fl.getEncoder().setPositionConversionFactor(Constants.DrivetrainConstants.kEncoderDistancePerPulse);
    spark_br.getEncoder().setPositionConversionFactor(Constants.DrivetrainConstants.kEncoderDistancePerPulse);
    spark_bl.getEncoder().setPositionConversionFactor(Constants.DrivetrainConstants.kEncoderDistancePerPulse);
    spark_fr.getEncoder().setVelocityConversionFactor(vel);
    spark_fl.getEncoder().setVelocityConversionFactor(vel);
    spark_br.getEncoder().setVelocityConversionFactor(vel);
    spark_bl.getEncoder().setVelocityConversionFactor(vel);

    gyro = new Pigeon2(Constants.pigeonID, "612Test");
    gyro.getConfigurator().apply(new Pigeon2Configuration());
    gyro.setYaw(0);
    gyro.reset();

    //CHECK THIS LATER!!
    // navx = new AHRS(NavXComType.kMXP_SPI);
    // navx.setAngleAdjustment(Constants.navxAngleOffset);
  }

    //Drives field relative
  public void drive(
      Translation2d translation, double rotation, boolean isOpenLoop, boolean robotRelative) {
    
      SwerveModuleState[] swerveModuleStates;
      if (robotRelative) {
        swerveModuleStates =
        Constants.swerveKinematics.toSwerveModuleStates(
          new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
      }
      else { //field relative
        swerveModuleStates =
        Constants.swerveKinematics.toSwerveModuleStates(
          ChassisSpeeds.fromFieldRelativeSpeeds(          // used to be get navx angle
                    translation.getX(), translation.getY(), rotation, getPigeonAngle()));
      } 
    
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.maxSpeed);

    for (SwerveModule mod : modules) {
      mod.setState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }


  public Rotation2d getPigeonAngle(){
    return gyro.getRotation2d();
    // used to be navx.getRotation2d();
  }

  public SwerveModulePosition[] getSwervePoses() {
      SwerveModulePosition[] mods = new SwerveModulePosition[4];
      for (SwerveModule mod : modules) {
        mods[mod.moduleNumber] = mod.getPosition();
      }
      return mods;
  }

  public void zeroGyro() {
    // navx.zeroYaw();
    gyro.reset();
  }


  public void resetAlignment() {
    for(SwerveModule mod : modules) {
      mod.resetToAbsolute();
    }
  }

  public static Mechanum getInstance(){
    if (swerveInstance == null){
      swerveInstance = new Mechanum();
    }
    return swerveInstance;
  }

  private Rotation2d getAngle() {
    return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
  }


  
  @Override
  public void periodic() {

    // SmartDashboard.putNumber("Current Angle", navx.getAngle());
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Gyro Angle", getAngle().getDegrees());
    for(SwerveModule mod : modules) {
      SmartDashboard.putNumber("Module " + mod.moduleNumber, mod.getAngle().getDegrees());
    }
  }
}