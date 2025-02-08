// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import com.revrobotics.*;

import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
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
  private boolean isCharacterizing = false;
  private double characterizationVolts = 0.0;

  public Pigeon2 gyro;
  // private AHRS navx;
  private final SparkMax spark_fl;
  private final SparkMax spark_fr;
  private final SparkMax spark_bl;
  private final SparkMax spark_br;
  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  private final MutDistance m_distance = Meters.mutable(0);
  private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);
  private final SysIdRoutine routine;

  /** Creates a new Swerve. */
  public Mecanum() {
// Creates a SysIdRoutine
    spark_fl = new SparkMax(Constants.DrivetrainConstants.SPARK_FL,MotorType.kBrushless);
    spark_fr = new SparkMax(Constants.DrivetrainConstants.SPARK_FR,MotorType.kBrushless);
    spark_bl = new SparkMax(Constants.DrivetrainConstants.SPARK_BL,MotorType.kBrushless);
    spark_br = new SparkMax(Constants.DrivetrainConstants.SPARK_BR,MotorType.kBrushless);
    routine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(voltage -> {
                spark_fl.setVoltage(voltage);
                spark_fr.setVoltage(voltage);
                spark_bl.setVoltage(voltage);
                spark_br.setVoltage(voltage);
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the left motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("spark_fl")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                          spark_fl.getBusVoltage() * spark_fl.getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(Constants.DrivetrainConstants.kEncoderDistancePerPulse*(spark_fl.getAbsoluteEncoder().getPosition()), Meters));
                log.motor("spark_fr")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                          spark_fr.getBusVoltage() * spark_fr.getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(Constants.DrivetrainConstants.kEncoderDistancePerPulse*(spark_fr.getAbsoluteEncoder().getPosition()), Meters));
                log.motor("spark_bl")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                          spark_bl.getBusVoltage() * spark_bl.getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(Constants.DrivetrainConstants.kEncoderDistancePerPulse*(spark_bl.getAbsoluteEncoder().getPosition()), Meters));
                log.motor("spark_br")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                          spark_br.getBusVoltage() * spark_br.getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(Constants.DrivetrainConstants.kEncoderDistancePerPulse*(spark_br.getAbsoluteEncoder().getPosition()), Meters));
                  }, this)
    );


    SparkMaxConfig sp = new SparkMaxConfig();
    sp.smartCurrentLimit(30);

    EncoderConfig conf = new EncoderConfig();
    conf.positionConversionFactor(Constants.DrivetrainConstants.kEncoderDistancePerPulse);
    conf.velocityConversionFactor(Constants.DrivetrainConstants.kEncoderDistancePerPulse/60);

    sp.apply(conf);
    
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


    public void runCharacterizationVolts(double volts) {
      isCharacterizing = true;
      characterizationVolts = volts;
    }
  
    /** Returns the average drive velocity in radians/sec. */
    public double getCharacterizationVelocity() {
      double driveVelocityAverage = 0.0;
      driveVelocityAverage += spark_fr.getEncoder().getVelocity();
      driveVelocityAverage += spark_fl.getEncoder().getVelocity();
      driveVelocityAverage += spark_br.getEncoder().getVelocity();
      driveVelocityAverage += spark_bl.getEncoder().getVelocity();

      return driveVelocityAverage / 4.0;
    }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
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


  public ChassisSpeeds getChassisSpeeds() {
    ChassisSpeeds result = Constants.DrivetrainConstants.m_kinematics.toChassisSpeeds(getSpeeds());
    return result;
  }


  public MecanumDriveWheelSpeeds getSpeeds() {
    var speeds = new MecanumDriveWheelSpeeds(spark_fl.get(),spark_fr.get(),spark_bl.get(),spark_br.get());
    return speeds;
  }

  public static Mecanum getInstance(){
    if (mechInstance == null){
      mechInstance = new Mecanum();
    }
    return mechInstance;
  }


  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pigeon",getPigeonAngle().getDegrees());
    if (isCharacterizing){
      spark_bl.setVoltage(characterizationVolts);
      spark_fr.setVoltage(characterizationVolts);
      spark_fl.setVoltage(characterizationVolts);
      spark_br.setVoltage(characterizationVolts);
    }
  }
  


  public void FieldOrientedDrive(double x, double y, double zRotation){
    if(Math.abs(x) < DEADZONE) x = 0;
    if(Math.abs(y) < DEADZONE) y = 0;
    if(Math.abs(zRotation) < DEADZONE) zRotation = 0;
    mech.driveCartesian(x, y, zRotation, getPigeonAngle());
  }
  

  public void RobotOrientedDrive(double x, double y, double zRot){
    if(Math.abs(x) < DEADZONE) x = 0;
    if(Math.abs(y) < DEADZONE) y = 0;
    if(Math.abs(zRot) < DEADZONE) zRot = 0;
    mech.driveCartesian(x, y, zRot);
  }
}