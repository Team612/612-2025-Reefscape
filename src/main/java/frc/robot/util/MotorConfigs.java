package frc.robot.util;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;


import frc.robot.Constants;


import com.revrobotics.spark.config.SparkMaxConfig;

public class MotorConfigs {
    public static SparkMaxConfig spark_pivot_configs;
    public static SparkMaxConfig spark_bag_configs;
    public static SparkMaxConfig elevator_pivot_configs;
    public static SparkMaxConfig climb_pivot_configs;
    public static SparkMaxConfig swerve_drive_configs;
    public static SparkMaxConfig swerve_angle_configs;

    public static CANdleConfiguration LED_configs;


    public MotorConfigs(){
        configureIntake();
        configureClimb();
        configureElevator();
        configureDrivetrain();
        configureLEDs();
                   
    }

    public static void configureIntake(){
        //pivot motor
        spark_pivot_configs = new SparkMaxConfig();
        spark_pivot_configs
            .inverted(Constants.IntakeConstants.pivotInverted)
            .smartCurrentLimit(Constants.IntakeConstants.pivotCurrentLimit)
            .idleMode(Constants.IntakeConstants.idleMode);
        
        spark_pivot_configs
            .encoder
                .positionConversionFactor(Constants.IntakeConstants.kPositionConversionFactor)
                .velocityConversionFactor(Constants.IntakeConstants.kVelocityConversionFactor);

        spark_pivot_configs
            .absoluteEncoder
                .positionConversionFactor(Constants.IntakeConstants.kPositionConversionFactor)
                .velocityConversionFactor(Constants.IntakeConstants.kVelocityConversionFactor);

        spark_pivot_configs
            .closedLoop
                .p(Constants.IntakeConstants.kP)
                .i(Constants.IntakeConstants.kI)
                .d(Constants.IntakeConstants.kD)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        
        spark_pivot_configs
            .closedLoop
                .maxMotion
                    .maxVelocity(Constants.IntakeConstants.maxVelocity)
                    .maxAcceleration(Constants.IntakeConstants.maxAcceleration)
                    .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
        

        //intake/outtake motor
        spark_bag_configs = new SparkMaxConfig();
        spark_bag_configs
            .inverted(Constants.IntakeConstants.bagInverted)
            .smartCurrentLimit(Constants.IntakeConstants.bagCurrentLimit)
            .idleMode(Constants.IntakeConstants.idleMode);

        spark_bag_configs
            .closedLoop
                .p(Constants.IntakeConstants.kP)
                .i(Constants.IntakeConstants.kI)
                .d(Constants.IntakeConstants.kD);
    }

    public static void configureElevator(){
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

    public static void configureClimb(){
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
        
        climb_pivot_configs
            .encoder
                .positionConversionFactor(Constants.ClimbConstants.kAngularPositionConversionFactor);
        
        
        climb_pivot_configs
            .absoluteEncoder
                .positionConversionFactor(Constants.ClimbConstants.kAngularPositionConversionFactor);


                
    }

    public static void configureDrivetrain(){
        swerve_drive_configs = new SparkMaxConfig();
        swerve_drive_configs
            .smartCurrentLimit(Constants.SwerveConstants.currentLimit);
        
        // swerve_motor_configs
        //     .encoder
        //         .positionConversionFactor(Constants.SwerveConstants.kPositionConversionFactor)
        //         .velocityConversionFactor(Constants.SwerveConstants.kVelocityConversionFactor);
        
        // swerve_motor_configs
        //     .absoluteEncoder
        //         .positionConversionFactor(Constants.SwerveConstants.kPositionConversionFactor)
        //         .velocityConversionFactor(Constants.SwerveConstants.kVelocityConversionFactor);
        swerve_drive_configs
            .closedLoop
                .p(Constants.SwerveConstants.drivekP)
                .i(Constants.SwerveConstants.drivekI)
                .d(Constants.SwerveConstants.drivekD);

        swerve_angle_configs

            .smartCurrentLimit(Constants.SwerveConstants.currentLimit);
        
        // swerve_angle_configs
        //     .closedLoop
        //         .positionWrappingEnabled(true)
        //         .positionWrappingInputRange(-Math.PI, Math.PI);
        

            
    }

    public void configureLEDs(){
        LED_configs = new CANdleConfiguration();
        LED_configs.stripType = LEDStripType.RGBW;
        LED_configs.vBatOutputMode = VBatOutputMode.Modulated;
    }


  



   
}