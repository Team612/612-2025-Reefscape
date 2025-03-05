package frc.robot;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  //constants
  private static final int constantXboxControllerPortNumber = 0;
  private static final int SPARK_FL = 2;
  private static final int SPARK_BR = 4;
  private static final int SPARK_BL = 3;
  private static final int SPARK_FR = 1;
  private static final int PigeonID = 0;
  private static final Double DEADZONE = 0.05;
  private static final int IntakePivotID = 6;
  private static final int IntakeBagID = 7;
  private static final double IntakePivotSpeed = 0.3;
  private static final double IntakeBagSpeed = 0.3;
  private static final int ElevatorID = 5;
  private static final int ServoID = 0;
  private static final int ClimbPivotID = 8;
  private static final double ClimbPivotSpeed = 0.1;
  private static final double Elevatorkp = 1;
  private static final double maxAutoElevatorSpeed = 0.5;
  private static final double coralStationPosition = 0.316;
  private static final double L2Position = 0.236;
  private static final double L3Position = 0.654;

  // instantiate things here
  private XboxController controller = new XboxController(constantXboxControllerPortNumber);
  private final static SparkMax spark_fl = new SparkMax(SPARK_FL,MotorType.kBrushless);
  private final static SparkMax spark_fr = new SparkMax(SPARK_FR,MotorType.kBrushless);
  private final static SparkMax spark_bl = new SparkMax(SPARK_BL,MotorType.kBrushless);
  private final static SparkMax spark_br = new SparkMax(SPARK_BR,MotorType.kBrushless);
  private final static SparkMax intakePivot = new SparkMax(IntakePivotID,MotorType.kBrushless);
  private final static SparkMax intakeBagSpeed = new SparkMax(IntakeBagID,MotorType.kBrushless);
  private final static SparkMax elevator = new SparkMax(ElevatorID,MotorType.kBrushless);
  private final static Servo servo = new Servo(ServoID);
  private final static SparkMax climbPivot = new SparkMax(ClimbPivotID,MotorType.kBrushless);
  private Pigeon2 gyro = new Pigeon2(PigeonID);
  private static MecanumDrive mech = new MecanumDrive(spark_fl, spark_bl, spark_fr, spark_br);
  private static int Lposition = -1;
  private PIDController elevatorController = new PIDController(Elevatorkp, 0, 0);
  
  // aplies deadband so the robot does not drift when you let go of the controller.
  @SuppressWarnings("deprecation")
  public Robot() {
    mech.setDeadband(DEADZONE);
    elevator.setInverted(true);
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("spark_fl Speed", spark_fl.get());
    SmartDashboard.putNumber("spark_fr Speed", spark_fr.get());
    SmartDashboard.putNumber("spark_bl Speed", spark_bl.get());
    SmartDashboard.putNumber("spark_br Speed", spark_br.get());
    SmartDashboard.putNumber("intakePivot Speed", intakePivot.get());
    SmartDashboard.putNumber("intakeBagSpeed Speed", intakeBagSpeed.get());
    SmartDashboard.putNumber("elevator Speed", elevator.get());
    SmartDashboard.putNumber("climbPivot Speed", climbPivot.get());
    SmartDashboard.putNumber("servo Position", servo.get());
    SmartDashboard.putNumber("elevator Position", elevator.getEncoder().getPosition());
    SmartDashboard.putNumber("intake Position", intakePivot.getEncoder().getPosition());
    SmartDashboard.putNumber("desired intake position", Lposition);

    if (elevator.getForwardLimitSwitch().isPressed()){
      elevator.getEncoder().setPosition(0);
    }
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    //resets the gyroscope to zero for field oriented if needed
    if (controller.getRightBumperButton())
      gyro.reset();
    
    //gets desired percent movement
    double x = -controller.getRawAxis(0);
    double y = -controller.getRawAxis(1);
    double zRot = controller.getRawAxis(4);

    //automatically calculates all wheel movements and aplies them to achieve holonomic drive
    mech.driveCartesian(y, x, zRot, gyro.getRotation2d());

    // controls pivot movement using X and B buttons
    if (controller.getXButton())
      intakePivot.set(IntakePivotSpeed);
    else if (controller.getBButton())
      intakePivot.set(-IntakePivotSpeed);
    else
      intakePivot.set(0);

    // controls intake bag movement using Y and A buttons
    if (controller.getYButton())
      intakeBagSpeed.set(IntakeBagSpeed);
    else if (controller.getAButton())
      intakeBagSpeed.set(-IntakeBagSpeed);
    else
      intakeBagSpeed.set(0);

    // controls elevator speed using both triggers
    // and adds pid control
    if ((controller.getRightTriggerAxis() != 0) || (controller.getLeftTriggerAxis() != 0))
      Lposition = -1;
    if (controller.getRawButtonPressed(7))
      Lposition = 2;
    if (controller.getRawButtonPressed(8))
      Lposition = 3;
    if (controller.getLeftBumperButtonPressed())
      Lposition = 1;
    
    double elevatorSpeed = 0.0;
    if (Lposition == -1){
      elevatorSpeed = controller.getRightTriggerAxis()-controller.getLeftTriggerAxis();
      elevator.set(elevatorSpeed);
    }
    else{
      if (Lposition == 1)
        elevatorSpeed = -elevatorController.calculate(elevator.getEncoder().getPosition(), coralStationPosition);
      else if (Lposition == 2)
        elevatorSpeed = -elevatorController.calculate(elevator.getEncoder().getPosition(), L2Position);
      else if (Lposition == 3)
        elevatorSpeed = -elevatorController.calculate(elevator.getEncoder().getPosition(), L3Position);
      if (elevatorSpeed > maxAutoElevatorSpeed)
        elevatorSpeed = maxAutoElevatorSpeed;
      if (elevatorSpeed < -maxAutoElevatorSpeed)
        elevatorSpeed = -maxAutoElevatorSpeed;
      elevator.set(elevatorSpeed);
    }

    // controls climb using the POV buttons
    // if (controller.getPOV() == 0)
    //   climbPivot.set(ClimbPivotSpeed);
    // else if (controller.getPOV() == 180)
    //   climbPivot.set(-ClimbPivotSpeed);
    // else
    //   climbPivot.set(0);
    // if (controller.getPOV() == 90)
    //   servo.set(1);
    // else if (controller.getPOV() == 270)
    //   servo.set(0);
    // else
    //   servo.set(0.5);
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