// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Payload;

public class Trapezoid extends Command {
  private Payload m_payload;
  private double targetPosition = 0.0; // Desired elevator position (meters)
  private double currentPosition = 0; // Current position of the elevator
  private double timer = 0;

  /** Creates a new ManualElevatorControl. */


  double kDt = 0.02;
  double kMaxVelocity = 0.3;
  double kMaxAccel = 0.3;
  
  private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAccel);
  private TrapezoidProfile.State previousState;
  private final ProfiledPIDController m_controller = new ProfiledPIDController(Constants.ElevatorConstants.kP, Constants.ElevatorConstants.kI, Constants.ElevatorConstants.kD, m_constraints);
  //ks: minimum voltage to overcome static friction. kG: minimum voltage to overcome gravity. Kv: idk Do kG before kS
  private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(Constants.ElevatorConstants.kS, Constants.ElevatorConstants.kG, Constants.ElevatorConstants.kV);



  public Trapezoid(Payload p) {
    m_payload = p;
    this.targetPosition = targetPosition; // Set the target position for the elevator
    addRequirements(m_payload);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_payload.setMotorSpeed(0); // Stop any previous movement
    currentPosition = m_payload.getPosition(); // Get current pos
    m_controller.reset(currentPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        TrapezoidProfile profile = new TrapezoidProfile(m_constraints);

        TrapezoidProfile.State currentState = new TrapezoidProfile.State(currentPosition, 0.0); // Current position and zero velocity
        TrapezoidProfile.State goalState = new TrapezoidProfile.State(targetPosition, 0.0); // Target position and zero velocity

      
        TrapezoidProfile.State profiledState = profile.calculate(timer, currentState, goalState);

     
        currentPosition = profiledState.position;

        double pidOutput = m_controller.calculate(currentPosition, profiledState.position);
        double feedforward = m_feedforward.calculate(profiledState.position, pidOutput);
        m_payload.setMotorSpeed(pidOutput + feedforward);

        timer += 0.02; 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_payload.setMotorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // System.out.println(ControlMap.gunner_controls.getRawAxis(0));
    return Math.abs(currentPosition - targetPosition) < 0.01;
  }
}
