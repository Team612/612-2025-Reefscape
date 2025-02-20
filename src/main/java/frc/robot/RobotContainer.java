// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Controls.ControlMap;
import frc.robot.commands.BagIn;
import frc.robot.commands.BagOut;
import frc.robot.commands.PivotIn;
import frc.robot.commands.PivotOut;
import frc.robot.commands.ElevatorCommands.ElevatorDown;
import frc.robot.commands.ElevatorCommands.ElevatorUp;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Payload;

public class RobotContainer {
  private final Payload m_pay;
  private final Intake m_intake;
  private final Command m_BagIn;
  private final Command m_BagOut;
  private final Command m_PivotIn;
  private final Command m_PivotOut;
  private final Command m_ElevatorUp;
  private final Command m_ElevatorDown;

  public RobotContainer() {
    m_pay = Payload.getInstance();
    m_intake = Intake.getInstance();
    m_BagIn = new BagIn(m_intake);
    m_BagOut =  new BagOut(m_intake);
    m_PivotOut = new PivotOut(m_intake);
    m_PivotIn = new PivotIn(m_intake);
    m_ElevatorUp = new ElevatorUp(m_pay);
    m_ElevatorDown = new ElevatorDown(m_pay);
    configureBindings();
  }

  private void configureBindings() {
    // ControlMap.driver_joystick.b().onTrue(m_resetPay);
    
    // ControlMap.driver_joystick.y().onTrue(m_incUp);
    // ControlMap.driver_joystick.a().onTrue(m_incDown);

    // ControlMap.driver_joystick.x().onTrue(m_GoToPos);

    ControlMap.gunner_joystick.a().whileTrue(m_ElevatorDown);
    ControlMap.gunner_joystick.y().whileTrue(m_ElevatorUp);

    ControlMap.gunner_joystick.rightTrigger().whileTrue(m_BagIn);
    ControlMap.gunner_joystick.leftTrigger().whileTrue(m_BagOut);

    ControlMap.gunner_joystick.rightBumper().whileTrue(m_PivotIn);
    ControlMap.gunner_joystick.leftBumper().whileTrue(m_PivotOut);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}