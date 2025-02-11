// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Controls.ControlMap;
import frc.robot.commands.Reset;
import frc.robot.commands.SwivelElevator;
import frc.robot.subsystems.Payload;

public class RobotContainer {
  private final Payload m_pay;
  private final Command m_movePay;
  private final Command m_resetPay;
  
  public RobotContainer() {
    m_pay = Payload.getInstance();

    m_movePay = new SwivelElevator(m_pay);
    m_resetPay = new Reset(m_pay);

    configureBindings();
  }

  private void configureBindings() {
    ControlMap.driver_joystick.a().toggleOnTrue(m_movePay);
    ControlMap.driver_joystick.b().toggleOnTrue(m_resetPay);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}