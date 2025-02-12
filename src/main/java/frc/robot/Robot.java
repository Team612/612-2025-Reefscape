package frc.robot;



import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

public class Robot extends TimedRobot {
 //constants
  private static final int constantXboxControllerPortNumber = 0;
  private static final int SPARK_FL = 2;
  private static final int SPARK_BR = 4;
  private static final int SPARK_BL = 3;
  private static final int SPARK_FR = 1;
  private static final int PigeonID = 0;
  private static final Double DEADZONE = 0.05;

  // instantiate things here
  private XboxController controller = new XboxController(constantXboxControllerPortNumber);
  private final static SparkMax spark_fl = new SparkMax(SPARK_FL,MotorType.kBrushless);
  private final static SparkMax spark_fr = new SparkMax(SPARK_FR,MotorType.kBrushless);
  private final static SparkMax spark_bl = new SparkMax(SPARK_BL,MotorType.kBrushless);
  private final static SparkMax spark_br = new SparkMax(SPARK_BR,MotorType.kBrushless);
  private Pigeon2 gyro = new Pigeon2(PigeonID);
  private static MecanumDrive mech = new MecanumDrive(spark_fl, spark_bl, spark_fr, spark_br);
  
  // aplies deadband so the robot does not drift when you let go of the controller.
  public Robot() {
    mech.setDeadband(DEADZONE);
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    //resets the gyroscope to zero for field oriented if needed
    if (controller.getXButtonPressed()){
      gyro.reset();
    }
    
    //gets desired percent movement
    double x = -controller.getRawAxis(0);
    double y = -controller.getRawAxis(1);
    double zRot = controller.getRawAxis(4);

    //automatically calculates all wheel movements and aplies them to achieve holonomic drive
    mech.driveCartesian(y, x, zRot, gyro.getRotation2d());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
