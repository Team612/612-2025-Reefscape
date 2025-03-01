// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.PivotCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.Motor;
import frc.robot.subsystems.Shoot;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  // private PivotCommand pivotCommand;
  private final CommandXboxController gunnerControls;
  public RobotContainer() {
    gunnerControls = new CommandXboxController(1); // 1 = gunner port
    // pivotCommand = new PivotCommand(new Motor(new TalonSRX(0)), 1);
    // Configure the button bindings
    configureBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureBindings() {
    // gunnerControls.a().onTrue(new SequentialCommandGroup(new PivotCommand(new Shoot(new SparkMax(0, MotorType.kBrushless)), 0), new ShootCommand(new Motor(new TalonSRX(0)), true, 1000, 1)));
    // gunnerControls.b().onTrue(new SequentialCommandGroup(new PivotCommand(new Shoot(new SparkMax(0, MotorType.kBrushless)), 1), new ShootCommand(new Motor(new TalonSRX(0)), true, 1000, 2)));
    // gunnerControls.x().onTrue(new SequentialCommandGroup(new PivotCommand(new Shoot(new SparkMax(0, MotorType.kBrushless)), 2), new ShootCommand(new Motor(new TalonSRX(0)), true, 1000, 3)));
    // gunnerControls.y().onTrue(new PivotCommand(new Shoot(new SparkMax(0, MotorType.kBrushless)), 3));
    gunnerControls.rightTrigger().whileTrue(new PivotCommand(new Shoot(new SparkMax(0, MotorType.kBrushless)),true));
    gunnerControls.leftTrigger().whileTrue(new PivotCommand(new Shoot(new SparkMax(0, MotorType.kBrushless)),false));
    gunnerControls.a().whileTrue(new ShootCommand(new Motor(new TalonSRX(0)), true, 1000, 1));
    gunnerControls.b().whileTrue(new ShootCommand(new Motor(new TalonSRX(0)), true, 1000, 2));
    gunnerControls.x().whileTrue(new ShootCommand(new Motor(new TalonSRX(0)), true, 1000, 3));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new Command() {
      
    };
  }
}