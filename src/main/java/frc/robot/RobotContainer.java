// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.estimator.PoseEstimator;
// import frc.robot.subsystems.PoseEstimator;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
// import frc.robot.util.TrajectoryConfiguration;
// import frc.robot.util.TrajectoryCreation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.robot.commands.AutoCommands.DriverCommands.ApriltagAlign;
import frc.robot.commands.AutoCommands.GunnerCommands.SetBagSpeedTimed;
// import frc.robot.commands.ClimbCommands.CloseServo;
// import frc.robot.commands.ClimbCommands.ManualClimbPivotControls;
// import frc.robot.commands.ClimbCommands.OpenServo;
// import frc.robot.commands.ClimbCommands.PivotClimbIn;
// import frc.robot.commands.ClimbCommands.PivotClimbOut;
import frc.robot.commands.DriveCommands.DefaultDrive;
import frc.robot.commands.DriveCommands.FieldRelativeDrive;
import frc.robot.commands.ElevatorCommands.ManualElevatorControl;
import frc.robot.commands.ElevatorCommands.SetElevatorPosition;
import frc.robot.commands.IntakeCommands.AutoOutCoral;
import frc.robot.commands.IntakeCommands.BagIn;
import frc.robot.commands.IntakeCommands.BagOut;
import frc.robot.commands.IntakeCommands.ManualIntakePivotControl;
import frc.robot.commands.IntakeCommands.SetIntakePivotPosition;
// import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Payload;
import frc.robot.subsystems.Swerve;
// import frc.robot.subsystems.Vision;
// import frc.robot.commands.ClimbCommands.ClimbConstantShift;
import frc.robot.util.ControlMap;
import frc.robot.util.MotorConfigs;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.commands.DriveCommands.RunOnTheFly;
import frc.robot.commands.DriveCommands.TrajectoryCreation;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.TrajectoryConfiguration;
import frc.robot.subsystems.Vision;

public class RobotContainer {
  private Payload m_payload;
  private Intake m_intake;
  // private Climb m_climb;
  private Swerve m_drivetrain;
  // private PoseEstimator m_poseEstimator;
  // private Vision m_vision;
  // private TrajectoryConfiguration m_trajConfigs;
  // private TrajectoryCreation m_trajCreation;
  private MotorConfigs m_motorConfigs = new MotorConfigs();

  private SendableChooser<Command> m_chooser;
  
  
  
  private Command m_BagIn;
  private Command m_BagOut;
  // private Command m_PivotIntakeIn;
  // private Command m_PivotIntakeOut;
  // private Command m_ElevatorUp;
  // private Command m_ElevatorDown;
  private Command m_defaultElevatorCommand;
  private Command m_defaultIntakeCommand;
  private Command m_defaultDrive;
  private Command m_fieldRelativeDrive;
  private Command m_closeServo;
  private Command m_openServo;
  private Command m_pivotClimbIn;
  private Command m_pivotClimbOut;

  private Command m_ClimbConstantShiftUp;
  private Command m_ClimbConstantShiftDown;

  private SequentialCommandGroup m_autoL1;
  private SequentialCommandGroup m_autoL2;
  private SequentialCommandGroup m_autoL3;
  private SequentialCommandGroup m_autoCoralStation;
  private SequentialCommandGroup m_outAndOpenClimb;
  private SequentialCommandGroup m_inAndClosedClimb;

  private final Swerve m_drivetrain;
  private final TrajectoryCreation m_traj;
  private final Vision m_vision;
  private final RunOnTheFly runOnTheFly;
  private final CommandXboxController driverControls;
  private final DefaultDrive m_defaultDrive;
  private final FieldRelativeDrive m_FieldRelativeDrive;
  private final PoseEstimator m_poseEstimator;
  private final TrajectoryConfiguration m_trajConfig;


  public RobotContainer() {
    m_payload = Payload.getInstance();
    m_intake = Intake.getInstance();
    // m_climb = Climb.getInstance();
    m_drivetrain = Swerve.getInstance();

    // m_poseEstimator = PoseEstimator.getPoseEstimatorInstance();
    // m_vision = Vision.getVisionInstance();
    // m_trajConfigs = TrajectoryConfiguration.getInstance();
    // m_trajCreation = TrajectoryCreation.getInstance();
    
    m_chooser = new SendableChooser<>();

    m_BagIn = new BagIn(m_intake);
    m_BagOut =  new BagOut(m_intake);
    // m_PivotIntakeOut = new PivotIntakeOut(m_intake, m_payload);
    // m_PivotIntakeIn = new PivotIntakeIn(m_intake,m_payload);
    // m_ElevatorUp = new ElevatorUp(m_payload);
    // m_ElevatorDown = new ElevatorDown(m_payload);
    m_defaultElevatorCommand = new ManualElevatorControl(m_payload);
    m_defaultIntakeCommand = new ManualIntakePivotControl(m_intake);

     m_poseEstimator = PoseEstimator.getPoseEstimatorInstance();
    m_vision = Vision.getVisionInstance();
    m_traj = new TrajectoryCreation();
    m_trajConfig = TrajectoryConfiguration.getInstance();
    runOnTheFly = new RunOnTheFly(m_drivetrain, m_poseEstimator, m_traj, m_vision, 0);


     driverControls = new CommandXboxController(Constants.SwerveConstants.driverPort);
    m_defaultDrive = new DefaultDrive( m_drivetrain,
            () -> -driverControls.getLeftY(),
            () -> -driverControls.getLeftX(),
            () -> -driverControls.getRightX());

    m_FieldRelativeDrive = new FieldRelativeDrive( m_drivetrain,
            () -> -driverControls.getLeftY(),
            () -> -driverControls.getLeftX(),
            () -> -driverControls.getRightX());


    // m_closeServo = new CloseServo(m_climb);
    // m_openServo = new OpenServo(m_climb);
    // m_pivotClimbIn = new PivotClimbIn(m_climb);
    // m_pivotClimbOut = new PivotClimbOut(m_climb);

    // m_ClimbConstantShiftUp = new ClimbConstantShift(0.05);
    // m_ClimbConstantShiftDown = new ClimbConstantShift(-0.05);


    configureCommands();
    configureBindings();
    configureDefaultCommand();
  
  }

  private void configureCommands(){

    m_autoCoralStation = new SequentialCommandGroup(new SetElevatorPosition(m_payload, m_intake,Constants.ElevatorConstants.CoralStationPosition)
    .andThen(new SetIntakePivotPosition(m_intake, m_payload, Constants.IntakeConstants.CoralStationPosition)));

    m_autoL1 = new SequentialCommandGroup(new SetElevatorPosition(m_payload,m_intake, Constants.ElevatorConstants.L1Position))
    .andThen(new AutoOutCoral(m_intake))
    .andThen(new SetElevatorPosition(m_payload,m_intake, Constants.ElevatorConstants.basePosition));

    m_autoL2 = new SequentialCommandGroup(new SetIntakePivotPosition(m_intake, m_payload, Constants.IntakeConstants.L2Position))
    .andThen(new SetElevatorPosition(m_payload, m_intake, Constants.ElevatorConstants.L2Position))
    .andThen(new SetBagSpeedTimed(m_intake));

    m_autoL3 =  new SequentialCommandGroup(new SetIntakePivotPosition(m_intake, m_payload, Constants.IntakeConstants.L3Position))
    .andThen(new SetElevatorPosition(m_payload, m_intake, Constants.ElevatorConstants.L3Position))
    .andThen(new SetBagSpeedTimed(m_intake));

    m_outAndOpenClimb = null; //new SequentialCommandGroup(m_pivotClimbOut).andThen(m_openServo);
    m_inAndClosedClimb = null;//new SequentialCommandGroup(m_closeServo).andThen(m_pivotClimbIn);

    m_chooser.addOption("Auto L1", m_autoL1);
    m_chooser.addOption("Auto L2", m_autoL2);
    m_chooser.addOption("Auto L3", m_autoL3);

    m_chooser.setDefaultOption("Nothing Selected", null);
    SmartDashboard.putData(m_chooser);
  }



  private void configureBindings() {
    ControlMap.driver_controls.leftBumper().onTrue(new InstantCommand(() -> m_drivetrain.zeroGyro()));
    ControlMap.driver_controls.rightBumper().toggleOnTrue(m_defaultDrive);
    ControlMap.driver_controls.b().onTrue(m_autoCoralStation);
    ControlMap.driver_controls.a().onTrue(m_autoL2);
    ControlMap.driver_controls.x().onTrue(m_autoL3);

    // ControlMap.driver_controls.leftTrigger().onTrue(new ApriltagAlign(
    //  m_poseEstimator,
    //  m_vision,
    //  m_trajCreation,
    //  -Constants.AutoConstants.xApriltagDisplacement ,
    //  -Constants.AutoConstants.yApriltagDisplacement));
     
    // ControlMap.driver_controls.rightTrigger().onTrue(new ApriltagAlign(
    //   m_poseEstimator,
    //   m_vision,
    //   m_trajCreation,
    //   -Constants.AutoConstants.xApriltagDisplacement,
    //   Constants.AutoConstants.yApriltagDisplacement));

    // ControlMap.driver_controls.y().onTrue(m_outAndOpenClimb);   
    // ControlMap.driver_controls.a().onTrue(m_inAndClosedClimb);
    // ControlMap.driver_controls.x().whileTrue(m_openServo);
    // ControlMap.driver_controls.b().whileTrue(m_closeServo);

    // ControlMap.driver_controls.povLeft().whileTrue(new ManualClimbPivotControls(m_climb, Constants.ClimbConstants.pivotSpeed));
    // ControlMap.driver_controls.povRight().whileTrue(new ManualClimbPivotControls(m_climb, -Constants.ClimbConstants.pivotSpeed));
    // ControlMap.driver_controls.povUp().onTrue(m_ClimbConstantShiftUp);
    // ControlMap.driver_controls.povDown().onTrue(m_ClimbConstantShiftDown);



    ControlMap.gunnerButton1.whileTrue(m_BagIn);
    ControlMap.gunnerButton2.onTrue(new SetIntakePivotPosition(m_intake,m_payload,Constants.IntakeConstants.L1Position));
    ControlMap.gunnerButton4.whileTrue(m_BagOut);
    ControlMap.gunnerButton5.onTrue(new SetIntakePivotPosition(m_intake,m_payload,Constants.IntakeConstants.L2Position));
    ControlMap.gunnerButton3.onTrue(new SetElevatorPosition(m_payload,m_intake, Constants.ElevatorConstants.L3Position));
    ControlMap.gunnerButton6.onTrue(new SetElevatorPosition(m_payload,m_intake, Constants.ElevatorConstants.L2Position));
    ControlMap.gunnerButton7.onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));
    ControlMap.gunnerButton8.onTrue(new SetIntakePivotPosition(m_intake, m_payload, Constants.IntakeConstants.CoralStationPosition));
    ControlMap.gunnerButton11.onTrue(new SetElevatorPosition(m_payload,m_intake, Constants.ElevatorConstants.CoralStationPosition));
    // ControlMap.gunnerButton12.onTrue(new SetElevatorPosition(m_payload,m_intake, Constants.ElevatorConstants.CoralStationPosition));
    // ControlMap.gunnerButton4.onTrue(new SetIntakePivotPosition(m_intake, m_payload, Constants.IntakeConstants.L1Position));

    
    // ControlMap.gunnerButton2.onTrue(new AutoOutCoral(m_intake));

    // ControlMap.gunnerButton7.onTrue(new SetElevatorPosition(m_payload, Constants.ElevatorConstants.CoralStationPosition));
    // ControlMap.gunnerButton8.onTrue(new SetElevatorPosition(m_payload, Constants.ElevatorConstants.L1Position));
    ControlMap.gunnerButton10.onTrue(new SetElevatorPosition(m_payload,m_intake, Constants.ElevatorConstants.L3Position));

    // ControlMap.gunnerButton11.onTrue(m_autoCoralStation);
   // ControlMap.gunnerButton12.onTrue(m_autoL1);
    ControlMap.gunnerButton13.onTrue(m_autoL2);
    ControlMap.gunnerButton14.onTrue(m_autoL3);

    //ControlMap.gunnerButton3.onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));


    // ControlMap.gunner_joystick.a().whileTrue(m_ElevatorUp);
    // ControlMap.gunner_joystick.y().whileTrue(m_ElevatorDown);
    // ControlMap.gunner_joystick.x().onTrue(m_pivotIn);
    // ControlMap.gunner_joystick.b().onTrue(m_pivotOut);

    // ControlMap.gunner_joystick.rightTrigger().whileTrue(m_BagIn);
    // ControlMap.gunner_joystick.leftTrigger().whileTrue(m_BagOut);

    // ControlMap.gunner_joystick.rightBumper().whileTrue(m_PivotIn);
    // ControlMap.gunner_joystick.leftBumper().whileTrue(m_PivotOut);

    


    // ControlMap.gunner_buttons.getRawButton(0);
  }

  public void configureDefaultCommand(){
    m_drivetrain.setDefaultCommand(m_defaultDrive);
    m_payload.setDefaultCommand(m_defaultElevatorCommand);
    m_intake.setDefaultCommand(m_defaultIntakeCommand);
  }

  public Command getAutonomousCommand() {
    return (m_chooser.getSelected() != null) ? m_chooser.getSelected() : Commands.print("No autonomous command configured");
  }
}