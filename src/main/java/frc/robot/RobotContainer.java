// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.PoseEstimator;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.IntakeCommands.BagIn;
import frc.robot.commands.IntakeCommands.BagOut;
import frc.robot.commands.IntakeCommands.PivotIn;
import frc.robot.commands.IntakeCommands.PivotOut;
import frc.robot.commands.IntakeCommands.SetPivotPosition;
import frc.robot.commands.AutoCommands.DriverCommands.ApriltagAlign;
import frc.robot.commands.AutoCommands.GunnerCommands.SetBagSpeedTimed;
import frc.robot.commands.ClimbCommands.CloseServo;
import frc.robot.commands.ClimbCommands.OpenServo;
import frc.robot.commands.DriveCommands.DefaultDrive;
import frc.robot.commands.DriveCommands.FieldRelativeDrive;
import frc.robot.commands.ElevatorCommands.ElevatorDown;
import frc.robot.commands.ElevatorCommands.ElevatorUp;
import frc.robot.commands.ElevatorCommands.ManualElevatorControl;
import frc.robot.commands.ElevatorCommands.SetElevatorPosition;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Mecanum;
import frc.robot.subsystems.Payload;
import frc.robot.subsystems.Vision;
import frc.robot.util.ControlMap;
import frc.robot.util.MotorConfigs;
import frc.robot.util.TrajectoryConfiguration;
import frc.robot.util.TrajectoryCreation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RobotContainer {
  private Payload m_payload;
  private Intake m_intake;
  private Climb m_climb;
  private Mecanum m_drivetrain;
  private PoseEstimator m_poseEstimator;
  private Vision m_vision;
  private TrajectoryConfiguration m_trajConfigs;
  private TrajectoryCreation m_trajCreation;
  private MotorConfigs m_motorConfigs = new MotorConfigs();

  private SendableChooser<Command> m_chooser;
  
  
  
  private Command m_BagIn;
  private Command m_BagOut;
  private Command m_PivotIn;
  private Command m_PivotOut;
  private Command m_ElevatorUp;
  private Command m_ElevatorDown;
  private Command m_defaultElevatorCommand;
  private Command m_defaultIntakeCommand;
  private Command m_defaultDrive;
  private Command m_fieldRelativeDrive;
  private Command m_closeServo;
  private Command m_openServo;
  private Command m_pivotIn;
  private Command m_pivotOut;
  private SequentialCommandGroup m_autoL1;
  private SequentialCommandGroup m_autoL2;
  private SequentialCommandGroup m_autoL3;


  public RobotContainer() {
    m_payload = Payload.getInstance();
    m_intake = Intake.getInstance();
    m_climb = Climb.getInstance();
    m_drivetrain = Mecanum.getInstance();
    m_poseEstimator = PoseEstimator.getPoseEstimatorInstance();
    m_vision = Vision.getVisionInstance();
    m_trajConfigs = TrajectoryConfiguration.getInstance();
    m_trajCreation = TrajectoryCreation.getInstance();
    
    m_chooser = new SendableChooser<>();

    m_BagIn = new BagIn(m_intake);
    m_BagOut =  new BagOut(m_intake);
    m_PivotOut = new PivotOut(m_intake);
    m_PivotIn = new PivotIn(m_intake);
    m_ElevatorUp = new ElevatorUp(m_payload);
    m_ElevatorDown = new ElevatorDown(m_payload);
    m_defaultElevatorCommand = new ManualElevatorControl(m_payload);

    m_defaultDrive = new DefaultDrive(m_drivetrain);
    m_fieldRelativeDrive = new FieldRelativeDrive(m_drivetrain);

    m_closeServo = new CloseServo(m_climb);
    m_openServo = new OpenServo(m_climb);
    m_pivotIn = new PivotIn(m_intake);
    m_pivotOut = new PivotOut(m_intake);

    configureCommands();
    configureBindings();
  
  }

  private void configureCommands(){
    m_autoL1 = new SequentialCommandGroup(new SetElevatorPosition(m_payload, Constants.ElevatorConstants.L1Position))
    .andThen(new SetPivotPosition(m_intake, Constants.IntakeConstants.L1Position))
    .andThen(new SetBagSpeedTimed(m_intake))
    .andThen(new SetElevatorPosition(m_payload, Constants.ElevatorConstants.basePosition));

    m_autoL2 = new SequentialCommandGroup(new SetElevatorPosition(m_payload, Constants.ElevatorConstants.L2Position))
    .andThen(new SetPivotPosition(m_intake, Constants.IntakeConstants.L2Position))
    .andThen(new SetBagSpeedTimed(m_intake))
    .andThen(new SetElevatorPosition(m_payload, Constants.ElevatorConstants.basePosition));

    m_autoL3 = new SequentialCommandGroup(new SetElevatorPosition(m_payload, Constants.ElevatorConstants.L3Position))
    .andThen(new SetPivotPosition(m_intake, Constants.IntakeConstants.L3Position))
    .andThen(new SetBagSpeedTimed(m_intake))
    .andThen(new SetElevatorPosition(m_payload, Constants.ElevatorConstants.basePosition));

    m_chooser.addOption("Auto L1", m_autoL1);
    m_chooser.addOption("Auto L2", m_autoL2);
    m_chooser.addOption("Auto L3", m_autoL3);

    m_chooser.setDefaultOption("Nothing Selected", null);
  }



  private void configureBindings() {
    ControlMap.driver_joystick.leftBumper().onTrue(new InstantCommand(() -> m_drivetrain.zeroGyro()));
    ControlMap.driver_joystick.rightBumper().toggleOnTrue(m_defaultDrive);

    ControlMap.driver_joystick.leftTrigger().onTrue(new ApriltagAlign(
     m_poseEstimator,
     m_vision,
     m_trajCreation,
     -Constants.AutoConstants.xApriltagDisplacement ,
     -Constants.AutoConstants.yApriltagDisplacement));
     
     ControlMap.driver_joystick.rightTrigger().onTrue(new ApriltagAlign(
      m_poseEstimator,
      m_vision,
      m_trajCreation,
      -Constants.AutoConstants.xApriltagDisplacement,
      Constants.AutoConstants.yApriltagDisplacement));


    ControlMap.gunner_joystick.a().whileTrue(m_ElevatorUp);
    ControlMap.gunner_joystick.y().whileTrue(m_ElevatorDown);
    ControlMap.gunner_joystick.x().onTrue(m_pivotIn);
    ControlMap.gunner_joystick.b().onTrue(m_pivotOut);

    ControlMap.gunner_joystick.rightTrigger().whileTrue(m_BagIn);
    ControlMap.gunner_joystick.leftTrigger().whileTrue(m_BagOut);

    ControlMap.gunner_joystick.rightBumper().whileTrue(m_PivotIn);
    ControlMap.gunner_joystick.leftBumper().whileTrue(m_PivotOut);


    // ControlMap.gunner_buttons.getRawButton(0);
  }

  public void configureDefaultCommand(){
    m_drivetrain.setDefaultCommand(m_fieldRelativeDrive);
    m_payload.setDefaultCommand(m_defaultElevatorCommand);
    m_intake.setDefaultCommand(m_defaultIntakeCommand);
  }

  public Command getAutonomousCommand() {
    return (m_chooser.getSelected() != null) ? m_chooser.getSelected() : Commands.print("No autonomous command configured");
  }
}