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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
public class Swerve extends SubsystemBase {

  private final SwerveModule mod0 = new SwerveModule(Constants.DrivetrainConstants.mod0AngleMotorID,Constants.DrivetrainConstants.mod0DriveMotorID,Constants.DrivetrainConstants.mod0CANcoderID,Constants.DrivetrainConstants.mod0EncoderOffset);
  private final SwerveModule mod1 = new SwerveModule(Constants.DrivetrainConstants.mod1AngleMotorID,Constants.DrivetrainConstants.mod1DriveMotorID,Constants.DrivetrainConstants.mod1CANcoderID,Constants.DrivetrainConstants.mod1EncoderOffset);
  private final SwerveModule mod2 = new SwerveModule(Constants.DrivetrainConstants.mod2AngleMotorID,Constants.DrivetrainConstants.mod2DriveMotorID,Constants.DrivetrainConstants.mod2CANcoderID,Constants.DrivetrainConstants.mod2EncoderOffset);
  private final SwerveModule mod3 = new SwerveModule(Constants.DrivetrainConstants.mod3AngleMotorID,Constants.DrivetrainConstants.mod3DriveMotorID,Constants.DrivetrainConstants.mod3CANcoderID,Constants.DrivetrainConstants.mod3EncoderOffset);
  private final Pigeon2 gyro = new Pigeon2(Constants.DrivetrainConstants.gyroID);
  public static Swerve swerveInstance = null; 
  private SwerveModulePosition[] modulePositions = {new SwerveModulePosition(),new SwerveModulePosition(),new SwerveModulePosition(),new SwerveModulePosition()};
  private SwerveDriveOdometry odometry = new SwerveDriveOdometry(Constants.DrivetrainConstants.swerveKinematics, new Rotation2d(), modulePositions);

  private RobotConfig config;

  @SuppressWarnings("removal")
  public Rotation2d getHeading(){
    return Rotation2d.fromDegrees(Math.IEEEremainder(gyro.getAngle(), 360));
  }

  public Swerve() {
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
    SwerveModuleState[] moduleStates = Constants.DrivetrainConstants.swerveKinematics.toSwerveModuleStates(c);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.DrivetrainConstants.MAX_SPEED);
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
  public double getGyro(){
    return gyro.getRotation2d().getDegrees();
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
    return Constants.DrivetrainConstants.swerveKinematics.toChassisSpeeds(moduleStates);
  }

  public static Swerve getInstance(){
    if (swerveInstance == null){
      swerveInstance = new Swerve();
    }
    return swerveInstance;
  }

  
  public double getAngleE(){
    return mod2.getCurrentAngle();
  }

  @Override
  public void periodic() {
    SwerveModulePosition[] tempModulePositions = {mod0.getCurrentWheelPosition(),mod1.getCurrentWheelPosition(),mod2.getCurrentWheelPosition(),mod3.getCurrentWheelPosition()};
    odometry.update(gyro.getRotation2d(), tempModulePositions);
    System.out.println(getAngleE());
  }
}