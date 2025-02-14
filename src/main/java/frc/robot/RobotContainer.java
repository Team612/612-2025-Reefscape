// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Controls.ControlMap;
import frc.robot.commands.OutInTake;
import frc.robot.commands.Reset;
import frc.robot.commands.Pivot;
import frc.robot.subsystems.Payload;
import frc.robot.commands.OutInTake;

public class RobotContainer {
  private final Payload m_pay;
  private final Command m_movePay;
  private final Command m_resetPay;
  private final Command m_incUp;
  private final Command m_incDown;
  
  public RobotContainer() {
    m_pay = Payload.getInstance();

    m_movePay = new Pivot(m_pay);
    m_resetPay = new Reset(m_pay);
    m_incUp = new OutInTake(0.05);
    m_incDown = new OutInTake(-0.05);
    configureBindings();
  }

  private void configureBindings() {
    m_pay.setDefaultCommand(m_movePay);
    ControlMap.driver_joystick.b().onTrue(m_resetPay);
    
    ControlMap.driver_joystick.y().onTrue(m_incUp);
    ControlMap.driver_joystick.a().onTrue(m_incDown);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}