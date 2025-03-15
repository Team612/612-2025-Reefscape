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
import frc.robot.util.SwerveModule;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {

  private final SwerveModule mod0 = new SwerveModule(Constants.SwerveConstants.mod0AngleMotorID,Constants.SwerveConstants.mod0DriveMotorID,Constants.SwerveConstants.mod0CANcoderID,Constants.SwerveConstants.mod0EncoderOffset);
  private final SwerveModule mod1 = new SwerveModule(Constants.SwerveConstants.mod1AngleMotorID,Constants.SwerveConstants.mod1DriveMotorID,Constants.SwerveConstants.mod1CANcoderID,Constants.SwerveConstants.mod1EncoderOffset);
  private final SwerveModule mod2 = new SwerveModule(Constants.SwerveConstants.mod2AngleMotorID,Constants.SwerveConstants.mod2DriveMotorID,Constants.SwerveConstants.mod2CANcoderID,Constants.SwerveConstants.mod2EncoderOffset);
  private final SwerveModule mod3 = new SwerveModule(Constants.SwerveConstants.mod3AngleMotorID,Constants.SwerveConstants.mod3DriveMotorID,Constants.SwerveConstants.mod3CANcoderID,Constants.SwerveConstants.mod3EncoderOffset);

  private final Pigeon2 gyro = new Pigeon2(Constants.SwerveConstants.gyroID);

  private SwerveModulePosition[] modulePositions = {new SwerveModulePosition(),new SwerveModulePosition(),new SwerveModulePosition(),new SwerveModulePosition()};
  private SwerveDriveOdometry odometry = new SwerveDriveOdometry(Constants.SwerveConstants.swerveKinematics, new Rotation2d(), modulePositions);

  private RobotConfig config;
  private static Swerve instance;


  public Swerve() {
    resetEncoders();
    gyro.reset();
    configureAutobuilder();

    
  }

  public void autoDrive(ChassisSpeeds c){
    SwerveModuleState[] moduleStates = Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(c);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.SwerveConstants.MAX_SPEED);
    setModuleState(moduleStates);
  }

  public void setModuleState(SwerveModuleState[] states){
    mod0.setState(states[0]);
    mod1.setState(states[1]);
    mod2.setState(states[2]);
    mod3.setState(states[3]);
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

  public Rotation2d getPigeonAngle() {
    return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
  }

  public Rotation2d getHeading(){
    return Rotation2d.fromDegrees(Math.IEEEremainder(gyro.getAngle(), 360));
  }

  public SwerveModulePosition[] getSwervePositions() {
    SwerveModulePosition[] mods = new SwerveModulePosition[4];
    mods[0] = mod0.getCurrentWheelPosition();
    mods[1] = mod1.getCurrentWheelPosition();
    mods[2] = mod2.getCurrentWheelPosition();
    mods[3] = mod3.getCurrentWheelPosition();
    
    return mods;
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
    return Constants.SwerveConstants.swerveKinematics.toChassisSpeeds(moduleStates);
  }

  public void configureAutobuilder(){
try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
    }
    AutoBuilder.configure(
      this::getPose,
      this::setPose,
      this::getCurrentChassisSpeeds,
      (speeds, feedforwards) -> autoDrive(speeds),
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
  public static Swerve getInstance(){
    if (instance == null){
      instance = new Swerve();
    }
    return instance;

  }

  @Override
  public void periodic() {
    SwerveModulePosition[] tempModulePositions = {mod0.getCurrentWheelPosition(),mod1.getCurrentWheelPosition(),mod2.getCurrentWheelPosition(),mod3.getCurrentWheelPosition()};
    odometry.update(gyro.getRotation2d(), tempModulePositions);
  }
}
