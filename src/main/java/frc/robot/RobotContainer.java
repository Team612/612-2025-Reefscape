// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Controls.ControlMap;
import frc.robot.commands.BagIn;
import frc.robot.commands.BagOut;
import frc.robot.commands.GoToPos;
import frc.robot.commands.IncrementPay;
import frc.robot.commands.PivotIn;
import frc.robot.commands.PivotOut;
import frc.robot.commands.Reset;
import frc.robot.commands.SwivelElevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Payload;

public class RobotContainer {
  private final Payload m_pay;
  private final Command m_movePay;
  private final Command m_resetPay;
  private final Command m_incUp;
  private final Command m_incDown;
  private final Command m_GoToPos;
  private final Intake m_intake;
  private final Command m_BagIn;
  private final Command m_BagOut;
  private final Command m_PivotIn;
  private final Command m_PivotOut;

  public RobotContainer() {
    m_pay = Payload.getInstance();
    m_intake = new Intake();
    m_BagIn = new BagIn(m_intake);
    m_BagOut =  new BagOut(m_intake);
    m_movePay = new SwivelElevator(m_pay);
    m_resetPay = new Reset(m_pay);
    m_incUp = new IncrementPay(0.05);
    m_incDown = new IncrementPay(-0.05);
    m_GoToPos = new GoToPos(m_pay);
    m_PivotOut = new PivotOut(m_intake);
    m_PivotIn = new PivotIn(m_intake);
    configureBindings();
  }

  private void configureBindings() {
    m_pay.setDefaultCommand(m_movePay);
    ControlMap.driver_joystick.b().onTrue(m_resetPay);
    
    ControlMap.driver_joystick.y().onTrue(m_incUp);
    ControlMap.driver_joystick.a().onTrue(m_incDown);

    ControlMap.driver_joystick.x().onTrue(m_GoToPos);

    ControlMap.driver_joystick.rightTrigger().whileTrue(m_BagIn);
    ControlMap.driver_joystick.leftTrigger().whileTrue(m_BagOut);

    ControlMap.driver_joystick.rightBumper().whileTrue(m_PivotIn);
    ControlMap.driver_joystick.leftBumper().whileTrue(m_PivotOut);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}