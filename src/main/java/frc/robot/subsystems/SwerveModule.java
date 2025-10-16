package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;

public class SwerveModule {
    private SparkMax driveMotor;
    private SparkMax steerMotor;
    public CANcoder angleEncoder;
    private double encoderOffset;
    private PIDController turnPIDController = new PIDController(DriveConstants.kp, 0, 0.0);

    public SwerveModule(int drivingMotorID, int steerMotorID, int angleEncoderID, double encoderOffset){
        driveMotor = new SparkMax(drivingMotorID,MotorType.kBrushless);
        steerMotor = new SparkMax(steerMotorID,MotorType.kBrushless);
        angleEncoder = new CANcoder(angleEncoderID);
        this.encoderOffset = encoderOffset;

        turnPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void setSwerveState(SwerveModuleState desiredState){
        desiredState.optimize(new Rotation2d(getCurrentAngle()));

        driveMotor.set(desiredState.speedMetersPerSecond);

        steerMotor.set(turnPIDController.calculate(getCurrentAngle(), desiredState.angle.getRadians()));
    }

    public double getCurrentVelocity(){
      return driveMotor.getEncoder().getVelocity() * DriveConstants.tickToMetersConstant;
    }

    public SwerveModulePosition getCurrentWheelPosition(){
      return new SwerveModulePosition(driveMotor.getEncoder().getPosition() * DriveConstants.tickToMetersConstant, new Rotation2d(getCurrentAngle()));
    }

    public double getCurrentAngle() {
        double rotations = angleEncoder.getAbsolutePosition().getValueAsDouble() - encoderOffset;
        if (rotations < -0.5)
          rotations += 1;
        return rotations * 2 * Math.PI;
      }
}
