// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import frc.robot.Constants;
import frc.robot.SwerveModuleConstants;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import com.ctre.phoenix6.hardware.CANcoder;
/** Add your docs here. */

public class SwerveModule {
    //drive motors
    private SparkMax driveMotor;
    private SparkClosedLoopController driveController;
    private RelativeEncoder driveEncoder;
    private SimpleMotorFeedforward driveFeedforward;

    //angle motors
    private SparkMax angleMotor;
    private SparkClosedLoopController angleController;
    private RelativeEncoder angleEncoder;

    //cancoder
    private CANcoder canCoder;
    
    //misc
    public int moduleNumber;
    private double lastAngle;
    private Rotation2d zeroAngle;

    public SwerveModule(int moduleNumber, SwerveModuleConstants constants, MotorConfigs configs){
        this.moduleNumber = moduleNumber;

        //drive motor initalization
        driveMotor = new SparkMax(constants.driveMotorID, MotorType.kBrushless);
        driveController = driveMotor.getClosedLoopController();
        driveEncoder = driveMotor.getEncoder();
        driveFeedforward = new SimpleMotorFeedforward(Constants.driveKS, Constants.driveKV, Constants.driveKA);

        //angle motor initalization
        angleMotor = new SparkMax(constants.angleMotorID, MotorType.kBrushless);
        angleController = angleMotor.getClosedLoopController();
        angleEncoder = angleMotor.getEncoder();

        //cancoder initalization
        canCoder = new CANcoder(constants.cancoderID);
        zeroAngle = constants.desiredAngle;

        //configures motor configs
        configure(configs);

        //saves the last angle
        lastAngle = getState().angle.getDegrees();
    }

    //set the current state of the wheel (motor speed, angle)
    public void setState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState.optimize(getState().angle);
        if (isOpenLoop){
            double speed = desiredState.speedMetersPerSecond / Constants.maxSpeed;
            driveController.setReference(speed, ControlType.kDutyCycle);
        }
        else {
            driveController.setReference(desiredState.speedMetersPerSecond, ControlType.kVelocity, ClosedLoopSlot.kSlot0, driveFeedforward.calculate(desiredState.speedMetersPerSecond));
        }

        //prevents jittering
        double angle = Math.abs(desiredState.speedMetersPerSecond) <= Constants.maxSpeed * 0.01
        ? lastAngle
        : desiredState.angle.getDegrees();
  
        angleController.setReference(angle, ControlType.kPosition);
        lastAngle = angle;
    }

    //aligns the wheels correcetly IN THE SAME DIRECTION
    public void resetToAbsolute(){
        double absolutePosition = (canCoder.getAbsolutePosition().getValueAsDouble() * 360.0) - zeroAngle.getDegrees();
        angleEncoder.setPosition(absolutePosition);
    }

    //gets the current relative angle of the wheels
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(angleEncoder.getPosition());
    }

    //gets the current absolute angle of the wheels
    public double getAbsoluteAngle(){
        return canCoder.getAbsolutePosition().getValueAsDouble() * 360.0;
    }


    //gets the current state of the wheels
    public SwerveModuleState getState(){
        double velocity = driveEncoder.getVelocity();
        Rotation2d rotation = new Rotation2d(Units.degreesToRadians(angleEncoder.getPosition()));

        return new SwerveModuleState(velocity, rotation);
    }


    //gets the current position of the wheel
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveMotor.getEncoder().getPosition(), getAngle());
    }

    //configures motor and cancoder parameters
    public void configure(MotorConfigs configs){
        driveMotor.configure(configs.driveMotorConfigs, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        angleMotor.configure(configs.angleMotorConfigs, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        driveEncoder.setPosition(0);
        resetToAbsolute();
        canCoder.getConfigurator().apply(configs.canCoderConfigs);
    } 
}
