package frc.robot;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Robot extends TimedRobot {

  // constants
  private int cim1ID = 15;
  private int cim2ID = 2;
  private int cim3ID = 11;
  private int cim4ID = 50;
  private int neo1ID = 1;
  private int neo2ID = 10;
  private int controllerPort = 0;

  //instantiation
  private TalonSRX cim1 = new TalonSRX(cim1ID);
  private TalonSRX cim2 = new TalonSRX(cim2ID);
  private TalonSRX cim3 = new TalonSRX(cim3ID);
  private TalonSRX cim4 = new TalonSRX(cim4ID);
  private SparkMax neo1 = new SparkMax(neo1ID, MotorType.kBrushless);
  private SparkMax neo2 = new SparkMax(neo2ID, MotorType.kBrushless);

  private CommandXboxController controller = new CommandXboxController(controllerPort);

  public Robot() {
    cim1.setInverted(true);
    cim2.setInverted(true);
  }

  public void resetMotors(){
    cim1.set(TalonSRXControlMode.PercentOutput,0);
    cim2.set(TalonSRXControlMode.PercentOutput,0);
    cim3.set(TalonSRXControlMode.PercentOutput,0);
    cim4.set(TalonSRXControlMode.PercentOutput,0);
    neo1.set(0);
    neo2.set(0);
  }

  // robot lifecycle
  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    resetMotors();
  }

  @Override
  public void teleopPeriodic() {
    double speed = -controller.getRightY();
    double turn = controller.getRightX();
    double deflator = 1;
    if (Math.abs(speed) + Math.abs(turn) > 1)
      deflator = 1/(Math.abs(speed) + Math.abs(turn));

    cim1.set(TalonSRXControlMode.PercentOutput,(speed + turn) * deflator);
    cim2.set(TalonSRXControlMode.PercentOutput,(speed + turn) * deflator);
    cim3.set(TalonSRXControlMode.PercentOutput,(speed - turn) * deflator);
    cim4.set(TalonSRXControlMode.PercentOutput,(speed - turn) * deflator);

    double intakeSpeed = controller.getLeftTriggerAxis();
    System.out.println(controller.getLeftTriggerAxis());
    neo1.set(-intakeSpeed);

    double outtakeSpeed = controller.getRightTriggerAxis();
    System.out.println(controller.getRightTriggerAxis());
    neo2.set(-outtakeSpeed);
  }

  @Override
  public void disabledInit() {
    resetMotors();
  }

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