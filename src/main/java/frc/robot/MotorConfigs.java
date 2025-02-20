package frc.robot;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class MotorConfigs {
    public static SparkMaxConfig spark_pivot_configs;
    public static TalonFXConfiguration talon_bag_configs;
    public static SparkMaxConfig elevator_pivot_configs;
    public static SparkMaxConfig climb_pivot_configs;


    
    public MotorConfigs(){
        configureIntake();
        configureClimb();
        configureElevator();
        configureIntake();            
    }

    public void configureIntake(){
        //pivot motor
        spark_pivot_configs = new SparkMaxConfig();
        spark_pivot_configs
            .inverted(Constants.PivotConstants.pivotInverted)
            .smartCurrentLimit(Constants.PivotConstants.pivotCurrentLimit)
            .idleMode(Constants.PivotConstants.idleMode);

        spark_pivot_configs
            .closedLoop
                .p(Constants.PivotConstants.kP)
                .i(Constants.PivotConstants.kI)
                .d(Constants.PivotConstants.kD);

        //intake/outtake motor
        talon_bag_configs = new TalonFXConfiguration();
        talon_bag_configs
        .CurrentLimits
            .SupplyCurrentLimitEnable = Constants.PivotConstants.bagCurrentLimitEnable;
        
        talon_bag_configs
        .CurrentLimits
            .SupplyCurrentLimit = Constants.PivotConstants.bagCurrentLimit;
    }

    public void configureElevator(){
        //elevator motor
        elevator_pivot_configs = new SparkMaxConfig();
        elevator_pivot_configs
            .inverted(Constants.ElevatorConstants.elevatorInverted)
            .smartCurrentLimit(Constants.ElevatorConstants.elevatorCurrentLimit)
            .idleMode(Constants.ElevatorConstants.idleMode);

        elevator_pivot_configs
            .encoder
                .positionConversionFactor(Constants.ElevatorConstants.kPositionConversionFactor)
                .velocityConversionFactor(Constants.ElevatorConstants.kVelocityConversionFactor);
        
        elevator_pivot_configs 
            .absoluteEncoder
                .positionConversionFactor(Constants.ElevatorConstants.kPositionConversionFactor)
                .velocityConversionFactor(Constants.ElevatorConstants.kVelocityConversionFactor);
        
        elevator_pivot_configs
            .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(Constants.ElevatorConstants.kP)
                .i(Constants.ElevatorConstants.kI)
                .d(Constants.ElevatorConstants.kD);

        elevator_pivot_configs
            .closedLoop
                .maxMotion
                    .allowedClosedLoopError(Constants.ElevatorConstants.marginOfError)
                    .maxVelocity(Constants.ElevatorConstants.maxElevatorSpeed)
                    .maxAcceleration(Constants.ElevatorConstants.maxElevatorAcceleration)
                    .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
        


    }

    public void configureClimb(){
        //climb pivot
        climb_pivot_configs = new SparkMaxConfig();
        climb_pivot_configs
            .inverted(Constants.ClimbConstants.inverted)
            .smartCurrentLimit(Constants.ClimbConstants.climbCurrentLimit)
            .idleMode(Constants.ClimbConstants.idleMode);

        climb_pivot_configs
            .closedLoop
                .p(Constants.ClimbConstants.kP)
                .i(Constants.ClimbConstants.kI)
                .d(Constants.ClimbConstants.kD);
                
    }

    

  



   
}
