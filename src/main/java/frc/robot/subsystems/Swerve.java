package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {

  private SwerveModule frontL;
  private SwerveModule frontR;
  private SwerveModule backL;
  private SwerveModule backR;

  private Pigeon2 gyro;
  private CANdle candle;

  private Rotation2d heading;

  private SwerveModulePosition[] modulePositions;
  private final Field2d field = new Field2d();

  private RobotConfig config;

  private boolean resetTrigger = false;
  private Pose2d resetPos = new Pose2d();

  public static final Transform3d frontCameraTransform =
  new Transform3d(
    new edu.wpi.first.math.geometry.Translation3d(
      edu.wpi.first.math.util.Units.inchesToMeters(7.5),
      edu.wpi.first.math.util.Units.inchesToMeters(0),
      edu.wpi.first.math.util.Units.inchesToMeters(8)),
    new edu.wpi.first.math.geometry.Rotation3d()
  );

  private static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

  private PhotonCamera frontCamera = new PhotonCamera(Constants.frontCameraName);
    private PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(
    aprilTagFieldLayout,
    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    frontCameraTransform
  );

  private Pose2d initialPose = new Pose2d(0, 0, new Rotation2d());
  private SwerveDrivePoseEstimator poseEstimator;

  public Swerve() {
    frontL = new SwerveModule(Constants.frontLDriveMotorID, Constants.frontLSteerMotorID, Constants.frontLCANcoderID, Constants.frontLEncoderOffset);
    frontR = new SwerveModule(Constants.frontRDriveMotorID, Constants.frontRSteerMotorID, Constants.frontRCANcoderID, Constants.frontREncoderOffset);
    backL = new SwerveModule(Constants.backLDriveMotorID, Constants.backLSteerMotorID, Constants.backLCANcoderID, Constants.backLEncoderOffset);
    backR = new SwerveModule(Constants.backRDriveMotorID, Constants.backRSteerMotorID, Constants.backRCANcoderID, Constants.backREncoderOffset);

    gyro = new Pigeon2(Constants.gyroID);

    candle = new CANdle(Constants.CANdleID);
    candle.animate(null);
    candle.setLEDs(255,255,255);

    modulePositions = new SwerveModulePosition[4];
    modulePositions[0] = new SwerveModulePosition();
    modulePositions[1] = new SwerveModulePosition();
    modulePositions[2] = new SwerveModulePosition();
    modulePositions[3] = new SwerveModulePosition();

    poseEstimator = new SwerveDrivePoseEstimator(
      Constants.swerveKinematics,
      new Rotation2d(),
      getModulePositions(),
      initialPose,
      // Odometry Standard Deviations, x y & z
      VecBuilder.fill(0.003, 0.003, 0.001),
      // Vision measurement std deviations
      VecBuilder.fill(0.07, 0.07, 0.08)
    );

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
      (speeds, feedforwards) -> autonomousDrive(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
      new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
        new PIDConstants(2.0, 0.0, 0.0), // Translation PID constants
        new PIDConstants(40.0, 0.0, 0.0) // Rotation PID constants
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
    chassisSpeed.vxMetersPerSecond *= Constants.metersPerSecondToPercent;
    chassisSpeed.vyMetersPerSecond *= Constants.metersPerSecondToPercent;
    chassisSpeed.omegaRadiansPerSecond *= Constants.radiansPerSecondToPercent;
    drive(chassisSpeed);
  }

  public void drive(ChassisSpeeds chassisSpeed){
    SwerveModuleState[] moduleStates = Constants.swerveKinematics.toSwerveModuleStates(chassisSpeed);
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
    return poseEstimator.getEstimatedPosition();
  }

  public void setPose(Pose2d p){
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
    return Constants.swerveKinematics.toChassisSpeeds(moduleStates);
  }

  public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] tempModulePositions = {frontL.getCurrentWheelPosition(),frontR.getCurrentWheelPosition(),backL.getCurrentWheelPosition(),backR.getCurrentWheelPosition()};
    return tempModulePositions;
  }

  public PathPlannerPath Forward(){
        Pose2d estimatedPose = poseEstimator.getEstimatedPosition();

        double x = estimatedPose.getX();
        double y = estimatedPose.getY();
        Rotation2d angle = estimatedPose.getRotation();

        List<Waypoint> bezierPoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(x, y, new Rotation2d(-90)),
            new Pose2d(x+1.0, y, new Rotation2d(-90))
        );

      PathConstraints constraints = new PathConstraints(1,1,1,1);

        // Create the path using the bezier points created above
        PathPlannerPath path = new PathPlannerPath(
            bezierPoints,
            constraints,
            null,
            new GoalEndState(0.0, angle) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );

        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping = true;
        return path;
      }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("FrontL", frontL.angleEncoder.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("FrontR", frontR.angleEncoder.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("BackL", backL.angleEncoder.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("BackR", backR.angleEncoder.getAbsolutePosition().getValueAsDouble());

    heading = Rotation2d.fromDegrees(Math.IEEEremainder(gyro.getYaw().getValueAsDouble(), 360));

    poseEstimator.update(
      gyro.getRotation2d(),
      getModulePositions()
    );

    PhotonPipelineResult result = frontCamera.getLatestResult();
    if (result.hasTargets()) {
      var estimatedPoseOptional = photonPoseEstimator.update(result);
      if (estimatedPoseOptional.isPresent()) {
        var estimatedPose = estimatedPoseOptional.get().estimatedPose;
        poseEstimator.addVisionMeasurement(
          estimatedPose.toPose2d(),
          result.getTimestampSeconds()
        );
      }
    }

    Pose2d pos = poseEstimator.getEstimatedPosition();

    SmartDashboard.putNumber("PosX", pos.getX());
    SmartDashboard.putNumber("PosY", pos.getY());
    SmartDashboard.putNumber("PosHeading", pos.getRotation().getDegrees());
    field.setRobotPose(pos);

    if (resetTrigger){
      gyro.reset();
      poseEstimator.resetPosition(
        heading,
        getModulePositions(),
        resetPos
      );
      resetTrigger = false;
    }

    SmartDashboard.putNumber("RobotHeading", heading.getDegrees());
  }
}