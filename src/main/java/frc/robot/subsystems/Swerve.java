// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.SparkConfigs;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.SwerveModule;

import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

// import com.studica.frc.AHRS;
// import com.studica.frc.AHRS.NavXComType;

public class Swerve extends SubsystemBase {
  private static Swerve swerveInstance;
  private SwerveModule[] modules;
  private SparkConfigs swerveModuleConfigs;
  private SwerveDriveOdometry odometry;
  public Pigeon2 gyro;

  // private AHRS navx;



  /** Creates a new Swerve. */
  public Swerve() {
    swerveModuleConfigs = new SparkConfigs();
    modules = new SwerveModule[] {
      new SwerveModule(0, Constants.Mod0.constants, swerveModuleConfigs),
      new SwerveModule(1, Constants.Mod1.constants, swerveModuleConfigs),
      new SwerveModule(2, Constants.Mod2.constants, swerveModuleConfigs),
      new SwerveModule(3, Constants.Mod3.constants, swerveModuleConfigs),
    };
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

  // used to be get navx angle
  public Rotation2d getPigeonAngle(){
    return gyro.getRotation2d();
    // used to be navx.getRotation2d();
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

  public static Swerve getInstance(){
    if (swerveInstance == null){
      swerveInstance = new Swerve();
    }
    return swerveInstance;
  }




  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Current Angle", navx.getAngle());
    // This method will be called once per scheduler run
  }
}
