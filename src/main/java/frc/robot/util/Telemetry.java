
package frc.robot.util;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

// import frc.robot.subsystems.Climb;
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
    Swerve m_drivetrain;

    NetworkTableInstance table;
    NetworkTable drivetrainData;
    NetworkTable payloadData;
    NetworkTable climbData;
    NetworkTable autonomousData;
  

    //drivetrain values
    GenericEntry pigeonAngle;
    GenericEntry FLangle;
    GenericEntry FRangle;
    GenericEntry BLangle;
    GenericEntry BRangle;
    GenericEntry FLAbsoluteAngle;
    GenericEntry FRAbsoluteAngle;
    GenericEntry BLAbsoluteAngle;
    GenericEntry BRAbsoluteAngle;

    public void initData(){
        m_drivetrain = Swerve.getInstance();
        table = NetworkTableInstance.getDefault();

        drivetrainData = table.getTable("Drivetrain Data");
        


        //drivetrain
        pigeonAngle = drivetrainData.getDoubleTopic("Robot Angle").getGenericEntry();
        FLangle = drivetrainData.getDoubleTopic("FL Angle").getGenericEntry();
        FRangle = drivetrainData.getDoubleTopic("FR Angle").getGenericEntry();
        BLangle = drivetrainData.getDoubleTopic("BL Angle").getGenericEntry();
        BRangle = drivetrainData.getDoubleTopic("BR Angle").getGenericEntry();
        FLAbsoluteAngle = drivetrainData.getDoubleTopic("FL Absolute Angle").getGenericEntry();
        FRAbsoluteAngle = drivetrainData.getDoubleTopic("FR Absolute Angle").getGenericEntry();
        BLAbsoluteAngle = drivetrainData.getDoubleTopic("BL Absolute Angle").getGenericEntry();
        BRAbsoluteAngle = drivetrainData.getDoubleTopic("BR Absolute Angle").getGenericEntry();
    }

    public void updateData(){
        //drivetrain
        pigeonAngle.setDouble(m_drivetrain.getPigeonAngle().getDegrees());
        FLangle.setDouble((m_drivetrain.getStates())[0].angle.getDegrees());
        FRangle.setDouble((m_drivetrain.getStates())[2].angle.getDegrees());
        BLangle.setDouble((m_drivetrain.getStates())[1].angle.getDegrees());
        BRangle.setDouble((m_drivetrain.getStates())[3].angle.getDegrees()); 
        FLAbsoluteAngle.setDouble((m_drivetrain.getAbsoluteAngles())[0]);
        FRAbsoluteAngle.setDouble((m_drivetrain.getAbsoluteAngles())[2]);
        BLAbsoluteAngle.setDouble((m_drivetrain.getAbsoluteAngles())[1]);
        BRAbsoluteAngle.setDouble((m_drivetrain.getAbsoluteAngles())[3]); 
    }
    }
  
