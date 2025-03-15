// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class mySwerveSubsystem extends SubsystemBase {

  private final mySwerveModule mod0 = new mySwerveModule(Constants.mod0AngleMotorID,Constants.mod0DriveMotorID,Constants.mod0CANcoderID,Constants.mod0EncoderOffset);
  private final mySwerveModule mod1 = new mySwerveModule(Constants.mod1AngleMotorID,Constants.mod1DriveMotorID,Constants.mod1CANcoderID,Constants.mod1EncoderOffset);
  private final mySwerveModule mod2 = new mySwerveModule(Constants.mod2AngleMotorID,Constants.mod2DriveMotorID,Constants.mod2CANcoderID,Constants.mod2EncoderOffset);
  private final mySwerveModule mod3 = new mySwerveModule(Constants.mod3AngleMotorID,Constants.mod3DriveMotorID,Constants.mod3CANcoderID,Constants.mod3EncoderOffset);
  private final Pigeon2 gyro = new Pigeon2(Constants.gyroID);
  private SwerveModulePosition[] modulePositions = {new SwerveModulePosition(),new SwerveModulePosition(),new SwerveModulePosition(),new SwerveModulePosition()};
  private SwerveDriveOdometry odometry = new SwerveDriveOdometry(Constants.swerveKinematics, new Rotation2d(), modulePositions);

  private RobotConfig config;

  @SuppressWarnings("removal")
  public Rotation2d getHeading(){
    return Rotation2d.fromDegrees(Math.IEEEremainder(gyro.getAngle(), 360));
  }

  public mySwerveSubsystem() {
    resetEncoders();
    gyro.reset();

    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
    }
    AutoBuilder.configure(
      this::getPose,
      this::setPose,
      this::getCurrentChassisSpeeds,
      (speeds, feedforwards) -> setRobotBassedOffFieldChassisSpeeds(speeds),
      new PPHolonomicDriveController(
              new PIDConstants(5, 0.0, 0.0), // Translation PID constants
              new PIDConstants(5, 0.0, 0.0) // Rotation PID constants
      ),
      config,
      () -> {
        // var alliance = DriverStation.getAlliance();
        // if (alliance.isPresent()) {
        //   return alliance.get() == DriverStation.Alliance.Red;
        // }
        return false;
      },
      this 
    );
  }

  public Object setRobotBassedOffFieldChassisSpeeds(ChassisSpeeds c){
    SwerveModuleState[] moduleStates = Constants.swerveKinematics.toSwerveModuleStates(c);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.MAX_SPEED);
    setAllModuleStates(moduleStates);
    return null;
  }

  public void setAllModuleStates(SwerveModuleState[] states){
    mod0.setMySwerveState(states[0]);
    mod1.setMySwerveState(states[1]);
    mod2.setMySwerveState(states[2]);
    mod3.setMySwerveState(states[3]);
  }

  public void resetEncoders(){
    odometry.resetPose(new Pose2d());
    mod0.resetEncoder();
    mod1.resetEncoder();
    mod2.resetEncoder();
    mod3.resetEncoder();
  }
  public void resetGyro(){
    gyro.reset();
  }
  public Pose2d getPose(){
    return new Pose2d(odometry.getPoseMeters().getX(),odometry.getPoseMeters().getY(),gyro.getRotation2d());
  }
  public void setPose(Pose2d p){
    odometry.resetPose(p);
  }
  public ChassisSpeeds getCurrentChassisSpeeds(){
    SwerveModuleState moduleStates[] = new SwerveModuleState[4];
    moduleStates[0] = new SwerveModuleState(mod0.getCurrentVelocity(), new Rotation2d(mod0.getCurrentAngle()));
    moduleStates[1] = new SwerveModuleState(mod1.getCurrentVelocity(), new Rotation2d(mod1.getCurrentAngle()));
    moduleStates[2] = new SwerveModuleState(mod2.getCurrentVelocity(), new Rotation2d(mod2.getCurrentAngle()));
    moduleStates[3] = new SwerveModuleState(mod3.getCurrentVelocity(), new Rotation2d(mod3.getCurrentAngle()));
    return Constants.swerveKinematics.toChassisSpeeds(moduleStates);
  }

  @Override
  public void periodic() {
    SwerveModulePosition[] tempModulePositions = {mod0.getCurrentWheelPosition(),mod1.getCurrentWheelPosition(),mod2.getCurrentWheelPosition(),mod3.getCurrentWheelPosition()};
    odometry.update(gyro.getRotation2d(), tempModulePositions);
  }
}
