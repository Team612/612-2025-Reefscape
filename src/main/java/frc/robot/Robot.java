// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.Telemetry;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private Telemetry m_telemetry;

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
    m_telemetry = new Telemetry();
  }

  public void robotInit() {
    m_telemetry.initData();
    if (!Preferences.containsKey("Pay Speed")){
      Preferences.setDouble("Pay Speed", Constants.ElevatorConstants.payloadspeed);
    }
    if (!Preferences.containsKey("Pivot Speed")){
      Preferences.setDouble("Pivot Speed", Constants.IntakeConstants.pivotspeed);
    }
    if (!Preferences.containsKey("Bag Speed")){
      Preferences.setDouble("Bag Speed", Constants.IntakeConstants.bagspeed);
    }
    if (!Preferences.containsKey("Intake Pivot kP")){
      Preferences.setDouble("Intake Pivot kP", Constants.IntakeConstants.kP);
    }
    if (!Preferences.containsKey("Intake Pivot kI")){
      Preferences.setDouble("Intake Pivot kI", Constants.IntakeConstants.kI);
    }
    if (!Preferences.containsKey("Intake Pivot kD")){
      Preferences.setDouble("Intake Pivot kD", Constants.IntakeConstants.kD);
    }
    if (!Preferences.containsKey("Elevator kP")){
      Preferences.setDouble("Elevator kP", Constants.ElevatorConstants.kP);
    }
    if (!Preferences.containsKey("Elevator kI")){
      Preferences.setDouble("Elevator kI", Constants.ElevatorConstants.kI);
    }
    if (!Preferences.containsKey("Elevator kD")){
      Preferences.setDouble("Elevator kD", Constants.ElevatorConstants.kD);
    }
  }

  @Override
  public void robotPeriodic() {
    m_telemetry.updateData();
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
