package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class Vision implements Subsystem {
    private static Vision visionInstance;
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);


    public Vision(){
        updateData();
    }

    public static Vision getInstance(){
        if (visionInstance == null){
          visionInstance = new Vision();
        }
        return visionInstance;
    }

    public void updateData() {
        x = tx.getDouble(0.0);
        y = ty.getDouble(0.0);
        area = ta.getDouble(0.0);
    }

    public Pose2d getAprilTagPose(){
        LimelightResults results = LimelightHelpers.getLatestResults(Constants.limeName);
        if (results.targets_Fiducials.length > 0) {
            LimelightTarget_Fiducial tag = results.targets_Fiducials[0]; 
            double tx = tag.tx;                 
            double ty = tag.ty;                  
            return new Pose2d(tx, ty, new Rotation2d((tag.getYaw()))); 
        }
        return new Pose2d();
    }

    public Pose2d returnTagPose(LimelightHelpers.LimelightTarget_Fiducial tag){
        return new Pose2d(tag.tx, tag.ty, new Rotation2d((tag.getYaw()))); 
    }

    public LimelightHelpers.LimelightTarget_Fiducial[] getAprilTags(){
        LimelightResults results = LimelightHelpers.getLatestResults(Constants.limeName);
        if (results.targets_Fiducials.length > 0) {
            return results.targets_Fiducials;
        }
        return new LimelightHelpers.LimelightTarget_Fiducial[0];
    }

    @Override
    public void periodic() {
        updateData();
        SmartDashboard.putNumber("April Tag x", x);
        SmartDashboard.putNumber("April Tag y", y);
    }
}