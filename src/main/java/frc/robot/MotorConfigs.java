package frc.robot;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.revrobotics.spark.config.SparkMaxConfig;

public class MotorConfigs {
    public static SparkMaxConfig spark_pivot_configs;
    public static TalonFXConfiguration talon_bag_configs;

    public static SparkMaxConfig elevator_pivot_configs;

    
    public MotorConfigs(){
        //PIVOT CONFIGURATIONS
        spark_pivot_configs = new SparkMaxConfig();
        spark_pivot_configs
            .inverted(Constants.PivotConstants.pivotInverted)
            .smartCurrentLimit(Constants.PivotConstants.pivotCurrentLimit)
            .idleMode(Constants.PivotConstants.idleMode);

        talon_bag_configs = new TalonFXConfiguration();
        talon_bag_configs
        .CurrentLimits
            .SupplyCurrentLimitEnable = Constants.PivotConstants.bagCurrentLimitEnable;
        
        talon_bag_configs
        .CurrentLimits
            .SupplyCurrentLimit = Constants.PivotConstants.bagCurrentLimit;


        elevator_pivot_configs = new SparkMaxConfig();
        elevator_pivot_configs
            .inverted(Constants.ElevatorConstants.elevatorInverted)
            .smartCurrentLimit(Constants.ElevatorConstants.elevatorCurrentLimit)
            .idleMode(Constants.ElevatorConstants.idleMode);


            
    }

    
    


    //intake configs

   
}
