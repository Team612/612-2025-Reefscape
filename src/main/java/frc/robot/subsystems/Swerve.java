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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Swerve extends SubsystemBase {
  
  private SwerveModule frontL;
  private SwerveModule frontR;
  private SwerveModule backL;
  private SwerveModule backR;

  private Pigeon2 gyro;

  private Rotation2d heading;

  private SwerveModulePosition[] modulePositions;
  private SwerveDriveOdometry odometry;
  private final Field2d field = new Field2d();

  private boolean resetTrigger = false;
  private Pose2d resetPos = new Pose2d();

  private RobotConfig config;

  public Swerve() {
    frontL = new SwerveModule(DriveConstants.frontLDriveMotorID, DriveConstants.frontLSteerMotorID, DriveConstants.frontLCANcoderID, DriveConstants.frontLEncoderOffset);
    frontR = new SwerveModule(DriveConstants.frontRDriveMotorID, DriveConstants.frontRSteerMotorID, DriveConstants.frontRCANcoderID, DriveConstants.frontREncoderOffset);
    backL = new SwerveModule(DriveConstants.backLDriveMotorID, DriveConstants.backLSteerMotorID, DriveConstants.backLCANcoderID, DriveConstants.backLEncoderOffset);
    backR = new SwerveModule(DriveConstants.backRDriveMotorID, DriveConstants.backRSteerMotorID, DriveConstants.backRCANcoderID, DriveConstants.backREncoderOffset);

    gyro = new Pigeon2(DriveConstants.gyroID);

    modulePositions = new SwerveModulePosition[4];
    modulePositions[0] = new SwerveModulePosition();
    modulePositions[1] = new SwerveModulePosition();
    modulePositions[2] = new SwerveModulePosition();
    modulePositions[3] = new SwerveModulePosition();
    odometry = new SwerveDriveOdometry(DriveConstants.swerveKinematics, new Rotation2d(), modulePositions);

    Shuffleboard.getTab("Drive").add("Field", field).withSize(6, 4);

    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
    }
    AutoBuilder.configure(
      this::getPose, // Robot pose supplier
      this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      this::autonomousDrive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
      new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
        new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
      ),
      config,
      () -> {
        // var alliance = DriverStation.getAlliance();
        // if (alliance.isPresent()) {
        //   return alliance.get() == DriverStation.Alliance.Red;
        // }
        return false;
      },
      this // Reference to this subsystem to set requirements
    );
  }

  public void autonomousDrive(ChassisSpeeds chassisSpeed){
    chassisSpeed.vxMetersPerSecond *= DriveConstants.metersPerSecondToPercent;
    chassisSpeed.vyMetersPerSecond *= DriveConstants.metersPerSecondToPercent;
    chassisSpeed.omegaRadiansPerSecond *= DriveConstants.radiansPerSecondToPercent;
    drive(chassisSpeed);
  }

  public void drive(ChassisSpeeds chassisSpeed){
    SwerveModuleState[] moduleStates = DriveConstants.swerveKinematics.toSwerveModuleStates(chassisSpeed);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, 1);

    frontL.setSwerveState(moduleStates[0]);
    frontR.setSwerveState(moduleStates[1]);
    backL.setSwerveState(moduleStates[2]);
    backR.setSwerveState(moduleStates[3]);
  }

  public Rotation2d getHeading(){
    return heading;
  }

  public Pose2d getPose(){
    return new Pose2d(odometry.getPoseMeters().getX(),odometry.getPoseMeters().getY(),heading);
  }

  public void setPose(Pose2d p){
    // p = new Pose2d(0,0,new Rotation2d(Math.PI));
    gyro.setYaw(p.getRotation().getDegrees());
    resetTrigger = true;
    resetPos = p;
  }

  public ChassisSpeeds getRobotRelativeSpeeds(){
    SwerveModuleState moduleStates[] = new SwerveModuleState[4];
    moduleStates[0] = new SwerveModuleState(frontL.getCurrentVelocity(), new Rotation2d(frontL.getCurrentAngle()));
    moduleStates[1] = new SwerveModuleState(frontR.getCurrentVelocity(), new Rotation2d(frontR.getCurrentAngle()));
    moduleStates[2] = new SwerveModuleState(backL.getCurrentVelocity(), new Rotation2d(backL.getCurrentAngle()));
    moduleStates[3] = new SwerveModuleState(backR.getCurrentVelocity(), new Rotation2d(backR.getCurrentAngle()));
    return DriveConstants.swerveKinematics.toChassisSpeeds(moduleStates);
  }

  @Override
  public void periodic() {
    heading = Rotation2d.fromDegrees(Math.IEEEremainder(gyro.getYaw().getValueAsDouble(), 360));

    SwerveModulePosition[] tempModulePositions = {frontL.getCurrentWheelPosition(),frontR.getCurrentWheelPosition(),backL.getCurrentWheelPosition(),backR.getCurrentWheelPosition()};
    odometry.update(gyro.getRotation2d(), tempModulePositions);
    if (resetTrigger){
      odometry.resetPose(resetPos);
      resetTrigger = false;
    }

    SmartDashboard.putNumber("RobotHeading", heading.getDegrees());
    SmartDashboard.putNumber("OdometryHeading",odometry.getPoseMeters().getRotation().getDegrees());
    SmartDashboard.putNumber("OdometryX",odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("OdometryY",odometry.getPoseMeters().getY());
    field.setRobotPose(odometry.getPoseMeters());
  }
}
