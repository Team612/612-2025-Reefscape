// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.MotorConfigs;


// import com.studica.frc.AHRS;
// import com.studica.frc.AHRS.NavXComType;

public class Payload extends SubsystemBase {
  private SparkMax elevatorMotor;
  private SparkClosedLoopController controller;
  private int timer = 0;



  // private SparkMax elevator2 = new SparkMax(Constants.elevatorID2, MotorType.kBrushless);

  // private SparkMax neoPivot = new SparkMax(Constants.neoPivotID, MotorType.kBrushless);
  private static Payload payloadInstance;
  // DigitalInput toplimitSwitch = new DigitalInput(Constants.toplimitSwitchID);
  // DigitalInput bottomlimitSwitch = new DigitalInput(Constants.bottomlimitSwitchID);

  double kDt = 0.02;
  double kMaxVelocity = 0.9;
  double kMaxAccel = 0.3;
  private final Timer m_timer = new Timer();
  
  private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAccel);
  private final ProfiledPIDController m_controller = new ProfiledPIDController(Constants.ElevatorConstants.kP, Constants.ElevatorConstants.kI, Constants.ElevatorConstants.kD, m_constraints);
  //ks: minimum voltage to overcome static friction. kG: minimum voltage to overcome gravity. Kv: idfk. Do kG before kS
  private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(Constants.ElevatorConstants.kS, Constants.ElevatorConstants.kG, Constants.ElevatorConstants.kV);
  private AnalogInput m_mag = new AnalogInput(Constants.ElevatorConstants.magID);
  private static DigitalInput m_limE = new DigitalInput(Constants.ElevatorConstants.limEID);


  public Payload() {
    elevatorMotor = new SparkMax(Constants.ElevatorConstants.elevatorID, MotorType.kBrushless);
    elevatorMotor.configure(MotorConfigs.elevator_pivot_configs, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    controller = elevatorMotor.getClosedLoopController();

  } 

public void setMotorSpeed(double speed) {
  if (m_limE.get() && speed > 0){
    elevatorMotor.set(0);
  }
  else {
  elevatorMotor.set(speed);
  }
}


public double magVoltage(){
  return m_mag.getVoltage();
}

public double magValue(){
  return m_mag.getValue();
}

// public Command profiledElevatorCommand(double distance) {
//    return startRun(
//            () -> {
//              // Restart timer so profile setpoints start at the beginning
//              m_timer.restart();
//              resetCount();
//            },
//            () -> {
//              // Current state never changes, so we need to use a timer to get the setpoints we need
//              // to be at
//              var currentTime = m_timer.get();
//              var currentSetpoint =
//                  m_profile.calculate(currentTime, new State(), new State(distance, 0));
//              var nextSetpoint =
//                  m_profile.calculate(
//                      currentTime + kDt, new State(), new State(distance, 0));
//              setStates(currentSetpoint, nextSetpoint);
//            })
//        .until(() -> m_profile.isFinished(0));
//  }
 
  public void setStates(
       TrapezoidProfile.State currentLeft,
       TrapezoidProfile.State nextLeft) {
     // Feedforward is divided by battery voltage to normalize it to [-1, 1]
     // SparkClosedLoopController m_loopy = elevatorMotor.getClosedLoopController();
       controller.setReference(nextLeft.position, ControlType.kPosition);
   }

public void setPosition(double position){
  State endGoal =  new State(position,0);
  if (m_limE.get() && position >= elevatorMotor.getEncoder().getPosition()){
    elevatorMotor.set(0);
  }
  else {
  // controller.setReference(-position, ControlType.kPosition);
  System.out.println(m_controller.calculate(getPosition(), endGoal));
  controller.setReference(-m_controller.calculate(getPosition(),endGoal), ControlType.kDutyCycle);
  }
  //m_controller.setGoal(-position);
  //elevatorMotor.setVoltage(m_controller.calculate(getPosition()) + m_feedforward.calculate(m_controller.getSetpoint().velocity));
//  controller.setReference(-position, ControlType.kMAXMotionPositionControl,ClosedLoopSlot.kSlot0, -m_feedforward.calculate(kMaxVelocity));
}

public void freezeMotors(){
  elevatorMotor.set(0);
}

public void resetPIDController(){
  m_controller.reset(getPosition());
}


public double getPosition(){
  return -elevatorMotor.getEncoder().getPosition();
}

public double getVelocity(){
  return elevatorMotor.getEncoder().getVelocity();
}

public SparkMax getElevatorMotor() {
  return elevatorMotor;
}

public double getVoltage(){
  return elevatorMotor.getAppliedOutput();
}

public static Payload getInstance(){
  if (payloadInstance == null){
    payloadInstance = new Payload();
  }
  return payloadInstance;
}

public static DigitalInput getSafetySwitch(){
  return m_limE;
}

public boolean isLimitPressed() {
  return elevatorMotor.getForwardLimitSwitch().isPressed();
}

  @Override
  public void periodic() {
    // System.out.println(getVoltage()); 
    // timer++;
    if (elevatorMotor.getForwardLimitSwitch().isPressed()){ //limit switch reads backwards; true when not pressed
      timer++;
      System.out.println("Limit switch is pressed, zero payload: " + timer);
      elevatorMotor.getEncoder().setPosition(0);
    }



    SmartDashboard.putNumber("Magnetic Sensor Reading (Voltage): ", magVoltage());
    SmartDashboard.putNumber("Magnetic Sensor Reading (Value): ", magValue());

    SmartDashboard.putBoolean("Elevator Limit Switch (Value): ", m_limE.get());
    // // Constants.ElevatorConstants.payloadspeed = Preferences.getDouble("Pay Speed", Constants.ElevatorConstants.payloadspeed);
    //    // // SmartDashboard.putNumber("Elevator 2 Velocity", elevator.getEncoder().getVelocity());
    // // SmartDashboard.putNumber("Elevator 2 position", elevator.getEncoder().getPosition());
    // Constants.ElevatorConstants.kP = Preferences.getDouble("Elevator Pivot kP", Constants.ElevatorConstants.kP);
    // Constants.ElevatorConstants.kI = Preferences.getDouble("Elevator Pivot kI", Constants.ElevatorConstants.kI);
    // Constants.ElevatorConstants.kD = Preferences.getDouble("Elevator Pivot kD", Constants.ElevatorConstants.kD);
  }
}