package frc.robot.util;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Mecanum;
import frc.robot.subsystems.Payload;
// import frc.robot.subsystems.PoseEstimator;
// import frc.robot.subsystems.Vision;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;

public class Telemetry {
    Mecanum m_drivetrain;
    Intake m_intake;
    Payload m_payload;
    Climb m_climb;
    // PoseEstimator m_poseEstimator;
    // Vision m_vision;

    ShuffleboardTab drivetrainTab;
    ShuffleboardTab payloadTab; //elevator + intake
    ShuffleboardTab climbTab;
    ShuffleboardTab autonTab;

    //drivetrain values
    GenericEntry pigeonAngle;
    GenericEntry sparkFlVelocity;
    GenericEntry sparkFrVelocity;
    GenericEntry sparkBlVelocity;
    GenericEntry sparkBrVelocity;

    //payload values
    GenericEntry elevatorVelocity;
    GenericEntry elevatorPosition;
    GenericEntry elevatorCurrentSetSpeed;
    GenericEntry elevatorkP;
    GenericEntry elevatorkI;
    GenericEntry elevatorkD;
    
    GenericEntry intakePivotVelocity;
    GenericEntry intakePivotPosition;
    GenericEntry intakeCurrentSetSpeed;
    GenericEntry intakekP;
    GenericEntry intakekI;
    GenericEntry intakekD;

    GenericEntry bagCurrentSetSpeed;
    GenericEntry bagVelocity;

    //climb values
    GenericEntry climbPivotVelocity;
    GenericEntry climbPivotPositon;
    GenericEntry climbCurrentSetSpeed;

    GenericEntry servoPosition;

    //auton values
    GenericEntry poseX;
    GenericEntry poseY;
    GenericEntry poseAngle;
    GenericEntry seeAprilTag;
    GenericEntry apriltagAmbiguity;
    GenericEntry apriltagID;
    GenericEntry apriltagX;
    GenericEntry apriltagY;

    public void initData(){
        m_drivetrain = Mecanum.getInstance();
        m_climb = Climb.getInstance();
        m_intake = Intake.getInstance();
        m_payload = Payload.getInstance();
        // m_poseEstimator = PoseEstimator.getPoseEstimatorInstance();
        // m_vision = Vision.getVisionInstance();

        drivetrainTab = Shuffleboard.getTab("Drivetrain");
        payloadTab = Shuffleboard.getTab("Payload");
        climbTab = Shuffleboard.getTab("Climb");
        autonTab = Shuffleboard.getTab("Autonomous");


        pigeonAngle = drivetrainTab.add("Robot Angle", 0.0).getEntry();
        sparkFlVelocity = drivetrainTab.add("FL Velocity",0.0).getEntry();
        sparkFrVelocity = drivetrainTab.add("Fr Velocity",0.0).getEntry();
        sparkBlVelocity = drivetrainTab.add("Bl Velocity",0.0).getEntry();
        sparkBrVelocity = drivetrainTab.add("Br Velocity",0.0).getEntry();
        elevatorCurrentSetSpeed = payloadTab.add("Elevator Current Set Speed",Constants.ElevatorConstants.payloadspeed)
        .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -1, "max", 1))
        .getEntry();

        elevatorVelocity = payloadTab.add("Elevator Velocity",0.0).getEntry();
        elevatorPosition = payloadTab.add("Elevator Position",0.0).getEntry();

        intakePivotPosition = payloadTab.add("Intake Position",0.0).getEntry();
        intakeCurrentSetSpeed = payloadTab.add("Intake Current Set Speed",Constants.IntakeConstants.pivotspeed)
        .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -1, "max", 1))
        .getEntry();
        bagCurrentSetSpeed = payloadTab.add("Bag Current Set Speed",Constants.IntakeConstants.bagspeed)
        .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -1, "max", 1))
        .getEntry();
        bagVelocity = payloadTab.add("Bag Velocity",0.0).getEntry();

        climbCurrentSetSpeed = climbTab.add("Climb Current Set Speed",Constants.ClimbConstants.pivotSpeed)
        .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -1, "max", 1))
        .getEntry();

        climbPivotPositon = climbTab.add("Climb Position",0.0).getEntry();
        climbPivotVelocity = climbTab.add("Climb Velocity",0.0).getEntry();
        servoPosition = climbTab.add("Servo Position",0.0).getEntry();

        poseX = autonTab.add("Pose X", 0.0).getEntry();
        poseY = autonTab.add("Pose Y", 0.0).getEntry();
        poseAngle = autonTab.add("Pose Angle",0.0).getEntry();
        seeAprilTag = autonTab.add("Sees Apriltag", false).getEntry();
        apriltagAmbiguity = autonTab.add("Apriltag Ambiguity",0.0).getEntry();
        apriltagX = autonTab.add("Apriltag X",0.0).getEntry();
        apriltagY = autonTab.add("Apriltag Y",0.0).getEntry();
    }

    public void updateData(){

        pigeonAngle.setDouble(m_drivetrain.getPigeonAngle().getDegrees());
        sparkFlVelocity.setDouble(m_drivetrain.getSparks()[0].getEncoder().getVelocity());
        sparkFrVelocity.setDouble(m_drivetrain.getSparks()[1].getEncoder().getVelocity());
        sparkBlVelocity.setDouble(m_drivetrain.getSparks()[2].getEncoder().getVelocity());
        sparkBrVelocity.setDouble(m_drivetrain.getSparks()[3].getEncoder().getVelocity());

        elevatorVelocity.setDouble(m_payload.getVelocity());
        elevatorPosition.setDouble(m_payload.getPosition());
        intakePivotPosition.setDouble(m_intake.getPivotPosition());
        bagVelocity.setDouble(m_intake.getBagSpeed());

        climbPivotPositon.setDouble(m_climb.getPivotPosition());
        climbPivotVelocity.setDouble(m_climb.getPivotVelocity());
        servoPosition.setDouble(m_climb.getServoPosition());

        

        Constants.ElevatorConstants.payloadspeed = (double) elevatorCurrentSetSpeed.get().getValue();
        Constants.IntakeConstants.pivotspeed = (double)  intakeCurrentSetSpeed.get().getValue();
        Constants.IntakeConstants.bagspeed = (double) bagCurrentSetSpeed.get().getValue();
        Constants.ClimbConstants.pivotSpeed = (double) climbCurrentSetSpeed.get().getValue();

        // poseX.setDouble(m_poseEstimator.getPose().getX());
        // poseY.setDouble(m_poseEstimator.getPose().getY());
        // poseAngle.setDouble(m_poseEstimator.getPose().getRotation().getDegrees());
        // seeAprilTag.setBoolean(m_vision.frontHasTag());
        // apriltagAmbiguity.setDouble((m_vision.frontHasTag()) ? m_vision.getFrontPipelineResult().getBestTarget().getPoseAmbiguity() : 0.0);
        // apriltagX.setDouble((m_vision.frontHasTag()) ? m_vision.getFrontPipelineResult().getBestTarget().getBestCameraToTarget().getX() : 0.0);
        // apriltagY.setDouble((m_vision.frontHasTag()) ? m_vision.getFrontPipelineResult().getBestTarget().getBestCameraToTarget().getY() : 0.0);
    }
}
