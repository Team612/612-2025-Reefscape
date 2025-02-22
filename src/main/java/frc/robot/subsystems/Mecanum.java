
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;

import static edu.wpi.first.units.Units.Meters;

import static edu.wpi.first.units.Units.Volts;


// import com.studica.frc.AHRS;
// import com.studica.frc.AHRS.NavXComType;

public class Mecanum extends SubsystemBase {
  private static Mecanum mechInstance;
  private static MecanumDrive mech;
  private final double DEADZONE = 0.1;
  private boolean isCharacterizing = false;
  private double characterizationVolts = 0.0;

private final SparkClosedLoopController driverControllerFL;
private final SparkClosedLoopController driverControllerFR;
private final SparkClosedLoopController driverControllerBL;
private final SparkClosedLoopController driverControllerBR;
  
  public Pigeon2 gyro;
  // private AHRS navx;
  private final SparkMax spark_fl;
  private final SparkMax spark_fr;
  private final SparkMax spark_bl;
  private final SparkMax spark_br;
  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  private final MutDistance m_distance = Meters.mutable(0);
//   private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);
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
                log.motor("drive_left")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                          ((spark_fl.getBusVoltage() + spark_bl.getBusVoltage())/2)* ((spark_fl.getAppliedOutput() + spark_bl.getAppliedOutput())/2) * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(Constants.DrivetrainConstants.kEncoderDistancePerPulse*(((spark_fl.getAbsoluteEncoder().getPosition() + spark_bl.getAbsoluteEncoder().getPosition())/2)), Meters));
                log.motor("drive_right")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                          ((spark_fr.getBusVoltage() + spark_br.getBusVoltage())/2) * ((spark_fr.getAppliedOutput() + spark_br.getAppliedOutput())/2) * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(Constants.DrivetrainConstants.kEncoderDistancePerPulse*((spark_fr.getAbsoluteEncoder().getPosition() + spark_br.getAbsoluteEncoder().getPosition())/2), Meters));
                  }, this)
    );

    driverControllerFL = spark_fl.getClosedLoopController();
    driverControllerFR = spark_fr.getClosedLoopController();
    driverControllerBL = spark_bl.getClosedLoopController();
    driverControllerBR = spark_br.getClosedLoopController();



    SparkMaxConfig sp = new SparkMaxConfig();
    sp.smartCurrentLimit(30);
    sp.closedLoop.p(Constants.DrivetrainConstants.kP);
    sp.closedLoop.i(Constants.DrivetrainConstants.kI);
    sp.closedLoop.d(Constants.DrivetrainConstants.kD);



    EncoderConfig conf = new EncoderConfig();
    conf.positionConversionFactor(Constants.DrivetrainConstants.kEncoderDistancePerPulse);
    conf.velocityConversionFactor(Constants.DrivetrainConstants.kEncoderDistancePerPulse/60);

    sp.apply(conf);
    
    spark_fr.configure(sp.inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    spark_br.configure(sp.inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    spark_fl.configure(sp.inverted(false), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    spark_bl.configure(sp.inverted(false), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    gyro = new Pigeon2(Constants.DrivetrainConstants.pigeonID);
    gyro.getConfigurator().apply(new Pigeon2Configuration());
    // gyro.setYaw(180);
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
    var speeds = new MecanumDriveWheelSpeeds(spark_fl.getEncoder().getVelocity(),spark_fr.getEncoder().getVelocity(),spark_bl.getEncoder().getVelocity(),spark_br.getEncoder().getVelocity());
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

  public void AutoDrive(ChassisSpeeds speeds){
    // double xPercent = speeds.vxMetersPerSecond;
    // double yPercent = speeds.vyMetersPerSecond;
    // double zPercent = speeds.omegaRadiansPerSecond;

    //  mech.driveCartesian(xPercent, yPercent, zPercent);

    MecanumDriveWheelSpeeds wheelSpeeds = Constants.DrivetrainConstants.m_kinematics.toWheelSpeeds(speeds);
    double frontleft = wheelSpeeds.frontLeftMetersPerSecond;
    double frontright = wheelSpeeds.frontRightMetersPerSecond;
    double backleft = wheelSpeeds.rearLeftMetersPerSecond;
    double backright = wheelSpeeds.rearRightMetersPerSecond;

    driverControllerFL.setReference(frontleft, ControlType.kVelocity, ClosedLoopSlot.kSlot0, Constants.DrivetrainConstants.kFeedforward.calculate(frontleft));
    driverControllerFR.setReference(frontright, ControlType.kVelocity, ClosedLoopSlot.kSlot0, Constants.DrivetrainConstants.kFeedforward.calculate(frontright));
    driverControllerBL.setReference(backleft, ControlType.kVelocity, ClosedLoopSlot.kSlot0, Constants.DrivetrainConstants.kFeedforward.calculate(backleft));
    driverControllerBR.setReference(backright, ControlType.kVelocity, ClosedLoopSlot.kSlot0, Constants.DrivetrainConstants.kFeedforward.calculate(backright));



  }
}
