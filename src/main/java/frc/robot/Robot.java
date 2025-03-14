package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  //CONSTANTS
  // used to instantiate swerve kinematics and calculate how to offset swerve kinematics
  private static final double trackWidth = 0.605;
  private static final double wheelBase = 0.605;
  
  // these are the max motor percent desired to be allocated to a certain dirrection or rotation, this works for any motor with a percent output 0-1
  private static final double xPercent = 1/2.0;
  private static final double yPercent = 1/2.0;
  private static final double zPercent = 1/2.0;

  // the swerve kinematics object only takes in radians per second and this will ofset those calculations for it to take in motor percent
  private static final double zNecessaryOffset = (zPercent)/(Math.sqrt((trackWidth/2)*(trackWidth/2)+(wheelBase/2)*(wheelBase/2)));

  // xbox port
  private static final int XboxPortNumber = 0;

  // gyro port
  private static final int gyroID = 0;

  // sets the minimum controller request percent
  private static final double Deadband = 0.05;

  // sets the proportional constant for the angle motor PID
  private static final double kp = 0.5;

  // swerve module 0 constants, front left
  // when the absolute encoder reads the 0.63 it is actually at 0
  private static final double mod0EncoderOffset = 0.63;
  private static final int mod0AngleMotorID = 7;
  private static final int mod0DriveMotorID = 6;
  private static final int mod0CANcoderID = 0;

  // swerve module 1 constants, front right
  // when the absolute encoder reads 0.735 it is actually at 0
  private static final double mod1EncoderOffset = 0.735;
  private static final int mod1AngleMotorID = 5;
  private static final int mod1DriveMotorID = 4;
  private static final int mod1CANcoderID = 2;

  // swerve module 2 constants, back left
  // when the absolute encoder reads 0.459 it is actually at 0
  private static final double mod2EncoderOffset = 0.459;
  private static final int mod2AngleMotorID = 1;
  private static final int mod2DriveMotorID = 8;
  private static final int mod2CANcoderID = 3;

  // swerve module 3 constants, back right
  // when the absolute encoder reads 0.2 it is actually at 0
  private static final double mod3EncoderOffset = 0.2;
  private static final int mod3AngleMotorID = 3;
  private static final int mod3DriveMotorID = 2;
  private static final int mod3CANcoderID = 1;

  // INSTANTIATING
  // xbox controller
  private XboxController controller = new XboxController(XboxPortNumber);

  // instantiating Swerve Modules
  private mySwerveModule mod0 = new mySwerveModule(mod0AngleMotorID,mod0DriveMotorID,mod0CANcoderID,mod0EncoderOffset);
  private mySwerveModule mod1 = new mySwerveModule(mod1AngleMotorID,mod1DriveMotorID,mod1CANcoderID,mod1EncoderOffset);
  private mySwerveModule mod2 = new mySwerveModule(mod2AngleMotorID,mod2DriveMotorID,mod2CANcoderID,mod2EncoderOffset);
  private mySwerveModule mod3 = new mySwerveModule(mod3AngleMotorID,mod3DriveMotorID,mod3CANcoderID,mod3EncoderOffset);

  // instantiating a Swerve Kinematics Object which will calculate our desired swerve module states
  private static final SwerveDriveKinematics swerveKinematics =
  new SwerveDriveKinematics(
      new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
      new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
      new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
      new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

  // instantiating a gyroscope so that our robot can drive in the field oriented mode
  Pigeon2 gyro = new Pigeon2(gyroID);

  public Robot() {
    SmartDashboard.putNumber("FrontL", mod0.angleEncoder.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("FrontR", mod1.angleEncoder.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("BackL", mod2.angleEncoder.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("BackR", mod3.angleEncoder.getAbsolutePosition().getValueAsDouble());
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    gyro.reset();
  }

  // Runs Periodically during the teleop phase of the match
  @Override
  public void teleopPeriodic() {
    // m/s desired speed
    double x = -controller.getRawAxis(1) * xPercent;
    double y = -controller.getRawAxis(0) * yPercent;
    double zRot = -controller.getRawAxis(4) * zNecessaryOffset;

    // resets heading for gyroscope
    if (controller.getAButtonPressed())
      gyro.reset();

    // sets controller output to 0 if it is below the deadband threshold
    // so the robot does not slowly drift when you let go of the controller
    if (Math.abs(x) < Deadband) x = 0;
    if (Math.abs(y) < Deadband) y = 0;
    if (Math.abs(zRot) < Deadband) zRot = 0;

    // updates the Chassis Object so we can plug it into the drive kinematics method
    // allows user to activate robot oriented by holding the right bumper 
    ChassisSpeeds chassisSpeed;
    if (!controller.getRightBumperButton()){
      @SuppressWarnings("removal")
      Rotation2d ourHeadingJustHowItWantsIt = Rotation2d.fromDegrees(Math.IEEEremainder(-gyro.getAngle(), 360));
      chassisSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(x,y,zRot, ourHeadingJustHowItWantsIt);
    }
    else{
      chassisSpeed = new ChassisSpeeds(x,y,zRot);
    }
    
    // automatically calculates what swerve module state every swerve module has to be at and adds it to this array
    SwerveModuleState[] moduleStates = swerveKinematics.toSwerveModuleStates(chassisSpeed);

    // slows down all of the motors by the same percent if a single motor goes over the max possible speed. so we dont lose control
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, 1);

    mod0.setMySwerveState(moduleStates[0]);
    mod1.setMySwerveState(moduleStates[1]);
    mod2.setMySwerveState(moduleStates[2]);
    mod3.setMySwerveState(moduleStates[3]);
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

  // nested class for swerve modules
  public class mySwerveModule{
    // data fields for each swerve module
    private SparkMax angleMotor;
    private SparkMax drivingMotor;
    public CANcoder angleEncoder;
    private double encoderOffset;
    private PIDController turnPIDController = new PIDController(kp, 0, 0);
  

    // constructor for the swerve module
    @SuppressWarnings("deprecation")
    public mySwerveModule(int angleMotorID, int drivingMotorID, int angleEncoderID, double encoderOffset){
      angleMotor = new SparkMax(angleMotorID,MotorType.kBrushless);
      drivingMotor = new SparkMax(drivingMotorID,MotorType.kBrushless);
      angleEncoder = new CANcoder(angleEncoderID);
      this.encoderOffset = encoderOffset;

      // sets driving motors to inverse so positive motor values make the robot go forward
      // drivingMotor.setInverted(true);

      // this important command allows the PID controller to determine the shortest way to reach a target
      // for example if the PID controller knows the wheel is currently at 3rad and wants to get to -3rad
      // it will just go forward the short way instead of backward the long way
      turnPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    // gets velocity of the wheel using the built in neo encoders
    public double getVelocity(){
      return drivingMotor.getEncoder().getVelocity();
    }

    public void slowlyDriveForward(){
      drivingMotor.set(0.1);
    }

    // this method takes in desired swerve module states and turns them into reality with motor inputs
    public void setMySwerveState(SwerveModuleState desiredState){
      // this SwerveModuleState method changes the desired state allowing the robot to run the wheels in reverse
      // for example if the front of the wheel is currently at pi/2 rad and it wants to get to -pi/2 rad
      // this method will change the desired state to -pi/2 and run the motors in reverse using the back of the wheel
      desiredState.optimize(new Rotation2d(getCurrentAngle()));

      // this transforms desired meters per second to motor percent output
      drivingMotor.set(desiredState.speedMetersPerSecond);

      // this sets the angle motor using pid control to ensure smooth turning
      angleMotor.set(turnPIDController.calculate(getCurrentAngle(), desiredState.angle.getRadians()));
    }

    // this returns the wheels current angle in the range (-pi,pi) from the CANcoder inputs
    public double getCurrentAngle() {
      double rotations = angleEncoder.getAbsolutePosition().getValueAsDouble() - encoderOffset;
      if (rotations < 0)
        rotations += 1;      
      if (rotations < 0.5)
        return rotations * 2 * Math.PI;
      else
        return (rotations-1) * 2 * Math.PI;
    }
  }
}