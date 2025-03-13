package frc.robot;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  //constants
  private static final int driverPort = 0;
  private static final int gunnerPort = 1;
  private static final int SPARK_FL = 2;
  private static final int SPARK_BR = 4;
  private static final int SPARK_BL = 3;
  private static final int SPARK_FR = 1;
  private static final int PigeonID = 0;
  private static final double DEADZONE = 0.05;
  private static final int IntakePivotID = 6;
  private static final double IntakePivotSpeed = 0.5;
  private static final int IntakeBagID = 7;
  private static final int ElevatorID = 5;
  private static final int ServoID = 0;
  private static final int ClimbPivotID = 8;
  private static final double ClimbPivotSpeed = 0.1;
  private static final double Elevatorkp = 1.8;
  private static final double maxAutoElevatorSpeed = 0.5;
  private static final double resetElevatorSpeed = 0.2;
  private static final double coralStationPosition = -0.316;
  private static final double L2Position = -0.236;
  private static final double L3Position = -0.654;

  // instantiate things here
  private XboxController driveController = new XboxController(driverPort);
  private XboxController gunController = new XboxController(gunnerPort);
  private final static SparkMax spark_fl = new SparkMax(SPARK_FL,MotorType.kBrushless);
  private final static SparkMax spark_fr = new SparkMax(SPARK_FR,MotorType.kBrushless);
  private final static SparkMax spark_bl = new SparkMax(SPARK_BL,MotorType.kBrushless);
  private final static SparkMax spark_br = new SparkMax(SPARK_BR,MotorType.kBrushless);
  private final static SparkMax intakePivot = new SparkMax(IntakePivotID,MotorType.kBrushless);
  private final static SparkMax intakeBag = new SparkMax(IntakeBagID,MotorType.kBrushed);
  private final static SparkMax elevator = new SparkMax(ElevatorID,MotorType.kBrushless);
  private final static Servo servo = new Servo(ServoID);
  private final static SparkMax climbPivot = new SparkMax(ClimbPivotID,MotorType.kBrushless);
  private Pigeon2 gyro = new Pigeon2(PigeonID);
  private static MecanumDrive mech = new MecanumDrive(spark_fl, spark_bl, spark_fr, spark_br);
  private static int Lposition = -1;
  private PIDController elevatorController = new PIDController(Elevatorkp, 0, 0);
  private SparkMaxConfig configs = new SparkMaxConfig();

  // private PhotonCamera

  
  
  // aplies deadband so the robot does not drift when you let go of the controller.
  @SuppressWarnings("deprecation")
  public Robot() {
    mech.setDeadband(DEADZONE);
    elevator.setInverted(true);    
    if (elevator.getForwardLimitSwitch().isPressed())
      elevator.getEncoder().setPosition(0);
    
      configs
      .limitSwitch.forwardLimitSwitchType(Type.kNormallyClosed).reverseLimitSwitchType(Type.kNormallyClosed);
      
      // configs
        // .closedLoop
        //   .pid(Elevatorkp, 0, 0);

    
    elevator.configure(configs, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakePivot.configure(configs, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // SparkClosedLoopController pidcontroller = elevator.getClosedLoopController();
    
    // pidcontroller.setReference(15, ControlType.kPosition);
  }
    

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("spark_fl Speed", spark_fl.get());
    SmartDashboard.putNumber("spark_fr Speed", spark_fr.get());
    SmartDashboard.putNumber("spark_bl Speed", spark_bl.get());
    SmartDashboard.putNumber("spark_br Speed", spark_br.get());
    SmartDashboard.putNumber("intakePivot Speed", intakePivot.get());
    SmartDashboard.putNumber("intakeBagSpeed Speed", intakeBag.get());
    SmartDashboard.putNumber("elevator Speed", elevator.get());
    SmartDashboard.putNumber("climbPivot Speed", climbPivot.get());
    
    //actually important stuff
    SmartDashboard.putNumber("intake Position", intakePivot.getEncoder().getPosition());
    SmartDashboard.putNumber("desired intake position", Lposition);
    SmartDashboard.putNumber("Elevator Velocity", elevator.getEncoder().getVelocity());
    SmartDashboard.putNumber("Elevator Error", elevator.getEncoder().getVelocity());
    SmartDashboard.putNumber("Elevator Position", elevator.getEncoder().getPosition());
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
    if (driveController.getRightBumperButton())
      gyro.reset();
    
    //gets desired percent movement
    double x = -driveController.getRawAxis(0);
    double y = -driveController.getRawAxis(1);
    double zRot = driveController.getRawAxis(4);

    //automatically calculates all wheel movements and aplies them to achieve holonomic drive
    mech.driveCartesian(y, x, zRot, gyro.getRotation2d());

    // controls elevator using gunner buttons
    // and adds pid control
    if ((Math.abs(gunController.getLeftY()) > 0.01) || (Math.abs(gunController.getRightX()) > 0.01))
      Lposition = -1;
    if (gunController.getAButton())
      Lposition = 1;
    if (gunController.getXButton())
      Lposition = 2;
    if (gunController.getYButton())
      Lposition = 3;
    if (gunController.getBButton())
      Lposition = 0;
    
    double elevatorSpeed = 0.0;
    if (Lposition == -1){
      elevator.set(gunController.getLeftY());
      intakePivot.set(gunController.getRightX());
    }
    if (Lposition == 0){
      intakePivot.set(IntakePivotSpeed);
      if (intakePivot.getForwardLimitSwitch().isPressed())
        elevator.set(resetElevatorSpeed);
      else
        elevator.set(0);
    }
    if (Lposition == 1){
      elevatorSpeed = elevatorController.calculate(elevator.getEncoder().getPosition(), L2Position);
      elevatorSpeed = applyMin(elevatorSpeed);
      intakePivot.set(IntakePivotSpeed);
      elevator.set(elevatorSpeed);
    }
    if (Lposition == 2){
      elevatorSpeed = elevatorController.calculate(elevator.getEncoder().getPosition(), coralStationPosition);
      elevatorSpeed = applyMin(elevatorSpeed);
      intakePivot.set(-IntakePivotSpeed);
      elevator.set(elevatorSpeed);
    }
    if (Lposition == 3){
      elevatorSpeed = elevatorController.calculate(elevator.getEncoder().getPosition(), L3Position);
      elevatorSpeed = applyMin(elevatorSpeed);
      intakePivot.set(IntakePivotSpeed);
      elevator.set(elevatorSpeed);
    }
    // controls intake bag motor with gunner variable inputs
    intakeBag.set(gunController.getRightTriggerAxis()-gunController.getLeftTriggerAxis());

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

  public static double applyMin(double elevatorSpeed){
    if (elevatorSpeed > maxAutoElevatorSpeed)
      elevatorSpeed = maxAutoElevatorSpeed;
    if (elevatorSpeed < -maxAutoElevatorSpeed)
      elevatorSpeed = -maxAutoElevatorSpeed;
    return elevatorSpeed;
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {} 
}