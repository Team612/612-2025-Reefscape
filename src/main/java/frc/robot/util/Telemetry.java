package frc.robot.util;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
// import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Mecanum;
import frc.robot.subsystems.Payload;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Vision;

import java.util.Map;

// import frc.robot.subsystems.PoseEstimator;
// import frc.robot.subsystems.Vision;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.PubSubOption;

public class Telemetry {
    Mecanum m_drivetrain;
    Intake m_intake;
    Payload m_payload;
    PoseEstimator m_PoseEstimator;
    Vision m_vision;
    // Climb m_climb;

    NetworkTableInstance table;
    NetworkTable drivetrainData;
    NetworkTable payloadData;
    NetworkTable climbData;
    NetworkTable autonomousData;
  

    //drivetrain values
    GenericEntry pigeonAngle;
    GenericEntry sparkFLVoltage;
    GenericEntry sparkFRVoltage;
    GenericEntry sparkBLVoltage;
    GenericEntry sparkBRVoltage;

    //payload values
    GenericEntry elevatorVelocity;
    GenericEntry elevatorPosition;
    GenericEntry elevatorCurrentUpSpeed;
    GenericEntry elevatorCurrentDownSpeed;
    GenericEntry elevatorkP;
    GenericEntry elevatorkI;
    GenericEntry elevatorkD;
    GenericEntry elevatorkG;
    GenericEntry elevatorkV;
    GenericEntry elevatorkS;
    
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

    GenericEntry servoStatus;

    //auton values
    GenericEntry poseX;
    GenericEntry poseY;
    GenericEntry poseAngle;
    GenericEntry seeAprilTagFront;
    GenericEntry seeAprilTagBack;
    GenericEntry apriltagAmbiguity;
    GenericEntry apriltagIDFront;
    GenericEntry apriltagIDBack;

    GenericEntry apriltagX;
    GenericEntry apriltagY;
    public void initData(){
        m_drivetrain = Mecanum.getInstance();
        m_payload = Payload.getInstance();
        m_intake = Intake.getInstance();
        // m_climb = Climb.getInstance();
        m_PoseEstimator = PoseEstimator.getPoseEstimatorInstance();
        m_vision = Vision.getVisionInstance();
        table = NetworkTableInstance.getDefault();

        drivetrainData = table.getTable("Drivetrain Data");
        payloadData = table.getTable("Payload Data");
        climbData = table.getTable("Climb Data");
        autonomousData = table.getTable("Autonomous Data");
        


        //drivetrain
        pigeonAngle = drivetrainData.getDoubleTopic("Robot Angle").getGenericEntry();
        sparkFRVoltage = drivetrainData.getDoubleTopic("FR Voltage").getGenericEntry();
        sparkFLVoltage = drivetrainData.getDoubleTopic("FL Voltage").getGenericEntry();
        sparkBRVoltage = drivetrainData.getDoubleTopic("BR Voltage").getGenericEntry();
        sparkBLVoltage = drivetrainData.getDoubleTopic("BL Voltage").getGenericEntry();


        //payload
        elevatorPosition = payloadData.getDoubleTopic("Payload Encoder Position (relative)").getGenericEntry();
        elevatorVelocity = payloadData.getDoubleTopic("Payload Velocity").getGenericEntry();
        intakePivotPosition = payloadData.getDoubleTopic("Intake Pivot Encoder Position (relative)").getGenericEntry();
        intakePivotVelocity = payloadData.getDoubleTopic("Intake Pivot Velocity").getGenericEntry();


        elevatorCurrentUpSpeed = payloadData.getDoubleTopic("Current Payload Up Speed (changeable)").getGenericEntry();
            elevatorCurrentUpSpeed.setDefaultDouble(Constants.ElevatorConstants.payloadUpSpeed);
        elevatorCurrentDownSpeed = payloadData.getDoubleTopic("Current Payload Down Speed (changeable)").getGenericEntry();
            elevatorCurrentDownSpeed.setDefaultDouble(Constants.ElevatorConstants.payloadDownSpeed);
        elevatorkP = payloadData.getDoubleTopic("Elevator kP (changeable)").getGenericEntry();
            elevatorkP.setDefaultDouble(Constants.ElevatorConstants.kP);
        elevatorkI = payloadData.getDoubleTopic("Elevator kI (changeable)").getGenericEntry();
            elevatorkI.setDefaultDouble(Constants.ElevatorConstants.kI);
        elevatorkD = payloadData.getDoubleTopic("Elevator kD (changeable)").getGenericEntry();
            elevatorkD.setDefaultDouble(Constants.ElevatorConstants.kD);
        elevatorkS = payloadData.getDoubleTopic("Elevator kS (changeable)").getGenericEntry();
            elevatorkS.setDefaultDouble(Constants.ElevatorConstants.kS);
        elevatorkG = payloadData.getDoubleTopic("Elevator kG (changeable)").getGenericEntry();
            elevatorkG.setDefaultDouble(Constants.ElevatorConstants.kG);
        elevatorkV = payloadData.getDoubleTopic("Elevator kV (changeable)").getGenericEntry();
            elevatorkV.setDefaultDouble(Constants.ElevatorConstants.kV);
        intakekP = payloadData.getDoubleTopic("Intake kP (changeable)").getGenericEntry();
            intakekP.setDefaultDouble(Constants.IntakeConstants.kP);
        intakekI = payloadData.getDoubleTopic("Intake kI (changeable)").getGenericEntry();
            intakekI.setDefaultDouble(Constants.IntakeConstants.kI);
        intakekD = payloadData.getDoubleTopic("Intake kD (changeable)").getGenericEntry();
            intakekD.setDefaultDouble(Constants.IntakeConstants.kD);
        bagCurrentSetSpeed = payloadData.getDoubleTopic("Current Bag Speed (changeable)").getGenericEntry();
            bagCurrentSetSpeed.setDefaultDouble(Constants.IntakeConstants.bagspeed);
        intakeCurrentSetSpeed = payloadData.getDoubleTopic("Current Intake Pivot Speed (changeable)").getGenericEntry();
            intakeCurrentSetSpeed.setDefaultDouble(Constants.IntakeConstants.pivotspeed);

        //climb
        climbPivotPositon = climbData.getDoubleTopic("Climb Pivot Encoder Position (relative)").getGenericEntry();
        climbPivotVelocity = climbData.getDoubleTopic("Climb Pivot Velocity").getGenericEntry();
        servoStatus = climbData.getBooleanTopic("Servo Closed?").getGenericEntry();

        climbCurrentSetSpeed = climbData.getDoubleTopic("Current Climb Pivot Speed (changeable)").getGenericEntry();
            climbCurrentSetSpeed.setDefaultDouble(Constants.ClimbConstants.pivotSpeed);

        //autonomous
        poseX = autonomousData.getDoubleTopic("Pose X").getGenericEntry();
        poseY = autonomousData.getDoubleTopic("Pose Y").getGenericEntry();
        poseAngle = autonomousData.getDoubleTopic("Pose Angle").getGenericEntry();
        seeAprilTagFront = autonomousData.getDoubleTopic("Sees Tag Front").getGenericEntry();
        seeAprilTagBack = autonomousData.getDoubleTopic("Sees Tag Back").getGenericEntry();
        apriltagAmbiguity = autonomousData.getDoubleTopic("Tag Ambiguity").getGenericEntry();
            apriltagAmbiguity.setDefaultDouble(0.0);        
        apriltagIDFront = autonomousData.getDoubleTopic("Tag ID Front").getGenericEntry();
            apriltagIDFront.setDefaultDouble(-1);
        apriltagIDBack = autonomousData.getDoubleTopic("Tag ID Back").getGenericEntry();
            apriltagIDBack.setDefaultDouble(-1);
        apriltagX = autonomousData.getDoubleTopic("Tag X").getGenericEntry();
            apriltagX.setDefaultDouble(-1);
        apriltagY = autonomousData.getDoubleTopic("Tag Y").getGenericEntry();
            apriltagY.setDefaultDouble(-1);
  
    
    }

    public void updateData(){
        //drivetrain
        pigeonAngle.setDouble(m_drivetrain.getPigeonAngle().getDegrees());
        sparkFLVoltage.setDouble(m_drivetrain.getSparks()[0].getOutputCurrent());
        sparkFRVoltage.setDouble(m_drivetrain.getSparks()[1].getOutputCurrent());
        sparkBLVoltage.setDouble(m_drivetrain.getSparks()[2].getOutputCurrent());
        sparkBRVoltage.setDouble(m_drivetrain.getSparks()[3].getOutputCurrent());

        //payload
        elevatorPosition.setDouble(m_payload.getPosition());
        elevatorVelocity.setDouble(m_payload.getVelocity());
        intakePivotPosition.setDouble(m_intake.getPivotPosition());
        intakePivotVelocity.setDouble(m_intake.getPivotSpeed());

        Constants.ElevatorConstants.kP = elevatorkP.getDouble(Constants.ElevatorConstants.kP);
        Constants.ElevatorConstants.kI = elevatorkI.getDouble(Constants.ElevatorConstants.kI);
        Constants.ElevatorConstants.kD = elevatorkD.getDouble(Constants.ElevatorConstants.kD);
        Constants.ElevatorConstants.kS = elevatorkS.getDouble(Constants.ElevatorConstants.kS);
        Constants.ElevatorConstants.kG = elevatorkG.getDouble(Constants.ElevatorConstants.kG);
        Constants.ElevatorConstants.kV = elevatorkV.getDouble(Constants.ElevatorConstants.kV);
        Constants.IntakeConstants.kP = intakekP.getDouble(Constants.IntakeConstants.kP);
        Constants.IntakeConstants.kI = intakekI.getDouble(Constants.IntakeConstants.kI);
        Constants.IntakeConstants.kD = intakekD.getDouble(Constants.IntakeConstants.kD);
        Constants.IntakeConstants.bagspeed = bagCurrentSetSpeed.getDouble(Constants.IntakeConstants.bagspeed);
        Constants.IntakeConstants.pivotspeed = intakeCurrentSetSpeed.getDouble(Constants.IntakeConstants.pivotspeed);
        Constants.ElevatorConstants.payloadUpSpeed = elevatorCurrentUpSpeed.getDouble(Constants.ElevatorConstants.payloadUpSpeed);
        Constants.ElevatorConstants.payloadDownSpeed = elevatorCurrentDownSpeed.getDouble(Constants.ElevatorConstants.payloadDownSpeed);

        //climb
        // climbPivotPositon.setDouble(m_climb.getPivotPosition());
        // climbPivotVelocity.setDouble(m_climb.getPivotVelocity());
        // servoStatus.setBoolean(m_climb.isServoClosed());

        Constants.ClimbConstants.pivotSpeed = climbCurrentSetSpeed.getDouble(Constants.ClimbConstants.pivotSpeed);

        //autonomous
        poseX.setDouble(m_PoseEstimator.getPose().getX());
        poseY.setDouble(m_PoseEstimator.getPose().getY());
        poseAngle.setDouble(m_PoseEstimator.getPose().getRotation().getDegrees());
        seeAprilTagFront.setBoolean(m_vision.frontHasTag());
        seeAprilTagBack.setBoolean(m_vision.backHasTag());
        apriltagIDFront.setDouble(m_vision.frontTagID());
        apriltagIDBack.setDouble(m_vision.backTagID());
     


        
    }
    }
    // PoseEstimator m_poseEstimator;
    // Vision m_vision;

    // ShuffleboardTab drivetrainTab;
    // ShuffleboardTab payloadTab; //elevator + intake
    // ShuffleboardTab climbTab;
    // ShuffleboardTab autonTab;

    // //drivetrain values
    // GenericEntry pigeonAngle;
    // GenericEntry sparkFlVelocity;
    // GenericEntry sparkFrVelocity;
    // GenericEntry sparkBlVelocity;
    // GenericEntry sparkBrVelocity;

    // //payload values
    // GenericEntry elevatorVelocity;
    // GenericEntry elevatorPosition;
    // GenericEntry elevatorCurrentSetSpeed;
    // GenericEntry elevatorkP;
    // GenericEntry elevatorkI;
    // GenericEntry elevatorkD;
    
    // GenericEntry intakePivotVelocity;
    // GenericEntry intakePivotPosition;
    // GenericEntry intakeCurrentSetSpeed;
    // GenericEntry intakekP;
    // GenericEntry intakekI;
    // GenericEntry intakekD;

    // GenericEntry bagCurrentSetSpeed;
    // GenericEntry bagVelocity;

    // //climb values
    // GenericEntry climbPivotVelocity;
    // GenericEntry climbPivotPositon;
    // GenericEntry climbCurrentSetSpeed;

    // GenericEntry servoPosition;

    // //auton values
    // GenericEntry poseX;
    // GenericEntry poseY;
    // GenericEntry poseAngle;
    // GenericEntry seeAprilTag;
    // GenericEntry apriltagAmbiguity;
    // GenericEntry apriltagID;
    // GenericEntry apriltagX;
    // GenericEntry apriltagY;

    // public void initData(){
    //     m_drivetrain = Mecanum.getInstance();
    //     m_climb = Climb.getInstance();
    //     m_intake = Intake.getInstance();
    //     m_payload = Payload.getInstance();
    //     // m_poseEstimator = PoseEstimator.getPoseEstimatorInstance();
    //     // m_vision = Vision.getVisionInstance();

    //     drivetrainTab = Shuffleboard.getTab("Drivetrain");
    //     payloadTab = Shuffleboard.getTab("Payload");
    //     climbTab = Shuffleboard.getTab("Climb");
    //     autonTab = Shuffleboard.getTab("Autonomous");


    //     pigeonAngle = drivetrainTab.add("Robot Angle", 0.0).getEntry();
    //     sparkFlVelocity = drivetrainTab.add("FL Velocity",0.0).getEntry();
    //     sparkFrVelocity = drivetrainTab.add("Fr Velocity",0.0).getEntry();
    //     sparkBlVelocity = drivetrainTab.add("Bl Velocity",0.0).getEntry();
    //     sparkBrVelocity = drivetrainTab.add("Br Velocity",0.0).getEntry();
    //     elevatorCurrentSetSpeed = payloadTab.add("Elevator Current Set Speed",Constants.ElevatorConstants.payloadspeed)
    //     .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -1, "max", 1))
    //     .getEntry();

    //     elevatorVelocity = payloadTab.add("Elevator Velocity",0.0).getEntry();
    //     elevatorPosition = payloadTab.add("Elevator Position",0.0).getEntry();

    //     intakePivotPosition = payloadTab.add("Intake Position",0.0).getEntry();
    //     intakeCurrentSetSpeed = payloadTab.add("Intake Current Set Speed",Constants.IntakeConstants.pivotspeed)
    //     .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -1, "max", 1))
    //     .getEntry();
    //     bagCurrentSetSpeed = payloadTab.add("Bag Current Set Speed",Constants.IntakeConstants.bagspeed)
    //     .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -1, "max", 1))
    //     .getEntry();
    //     bagVelocity = payloadTab.add("Bag Velocity",0.0).getEntry();

    //     climbCurrentSetSpeed = climbTab.add("Climb Current Set Speed",Constants.ClimbConstants.pivotSpeed)
    //     .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -1, "max", 1))
    //     .getEntry();

    //     climbPivotPositon = climbTab.add("Climb Position",0.0).getEntry();
    //     climbPivotVelocity = climbTab.add("Climb Velocity",0.0).getEntry();
    //     servoPosition = climbTab.add("Servo Position",0.0).getEntry();

    //     poseX = autonTab.add("Pose X", 0.0).getEntry();
    //     poseY = autonTab.add("Pose Y", 0.0).getEntry();
    //     poseAngle = autonTab.add("Pose Angle",0.0).getEntry();
    //     seeAprilTag = autonTab.add("Sees Apriltag", false).getEntry();
    //     apriltagAmbiguity = autonTab.add("Apriltag Ambiguity",0.0).getEntry();
    //     apriltagX = autonTab.add("Apriltag X",0.0).getEntry();
    //     apriltagY = autonTab.add("Apriltag Y",0.0).getEntry();
    // }

    // public void updateData(){

    //     pigeonAngle.setDouble(m_drivetrain.getPigeonAngle().getDegrees());
    //     sparkFlVelocity.setDouble(m_drivetrain.getSparks()[0].getEncoder().getVelocity());
    //     sparkFrVelocity.setDouble(m_drivetrain.getSparks()[1].getEncoder().getVelocity());
    //     sparkBlVelocity.setDouble(m_drivetrain.getSparks()[2].getEncoder().getVelocity());
    //     sparkBrVelocity.setDouble(m_drivetrain.getSparks()[3].getEncoder().getVelocity());

    //     elevatorVelocity.setDouble(m_payload.getVelocity());
    //     elevatorPosition.setDouble(m_payload.getPosition());
    //     intakePivotPosition.setDouble(m_intake.getPivotPosition());
    //     bagVelocity.setDouble(m_intake.getBagSpeed());

    //     climbPivotPositon.setDouble(m_climb.getPivotPosition());
    //     climbPivotVelocity.setDouble(m_climb.getPivotVelocity());
    //     servoPosition.setDouble(m_climb.getServoPosition());

        

    //     Constants.ElevatorConstants.payloadspeed = (double) elevatorCurrentSetSpeed.get().getValue();
    //     Constants.IntakeConstants.pivotspeed = (double)  intakeCurrentSetSpeed.get().getValue();
    //     Constants.IntakeConstants.bagspeed = (double) bagCurrentSetSpeed.get().getValue();
    //     Constants.ClimbConstants.pivotSpeed = (double) climbCurrentSetSpeed.get().getValue();

    //     // poseX.setDouble(m_poseEstimator.getPose().getX());
    //     // poseY.setDouble(m_poseEstimator.getPose().getY());
    //     // poseAngle.setDouble(m_poseEstimator.getPose().getRotation().getDegrees());
    //     // seeAprilTag.setBoolean(m_vision.frontHasTag());
    //     // apriltagAmbiguity.setDouble((m_vision.frontHasTag()) ? m_vision.getFrontPipelineResult().getBestTarget().getPoseAmbiguity() : 0.0);
    //     // apriltagX.setDouble((m_vision.frontHasTag()) ? m_vision.getFrontPipelineResult().getBestTarget().getBestCameraToTarget().getX() : 0.0);
    //     // apriltagY.setDouble((m_vision.frontHasTag()) ? m_vision.getFrontPipelineResult().getBestTarget().getBestCameraToTarget().getY() : 0.0);
    // }

