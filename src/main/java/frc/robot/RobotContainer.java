// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.util.Units;
// import frc.robot.subsystems.PoseEstimator;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoCommands.GunnerCommands.SetBagSpeedInTimed;
import frc.robot.commands.AutoCommands.GunnerCommands.SetBagSpeedTimed;
import frc.robot.commands.DriveCommands.ArcadeDrive;
import frc.robot.commands.ElevatorCommands.ManualElevatorControl;
import frc.robot.commands.ElevatorCommands.SetElevatorPosition;
import frc.robot.commands.ElevatorCommands.ZeroElevator;
import frc.robot.commands.IntakeCommands.AutoOutCoral;
import frc.robot.commands.IntakeCommands.BagIn;
import frc.robot.commands.IntakeCommands.BagOut;
import frc.robot.commands.IntakeCommands.ManualIntakePivotControl;
import frc.robot.commands.IntakeCommands.SetIntakePivotPosition;
import frc.robot.commands.IntakeCommands.ZeroIntake;
import frc.robot.subsystems.Bag;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Payload;
// import frc.robot.subsystems.PoseEstimator;
// import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.util.ControlMap;
import frc.robot.util.MotorConfigs;
import frc.robot.util.PathPlannerUtil;

public class RobotContainer {
  private Payload m_payload;
  private Intake m_intake;
  private Bag m_bag; 
  // private PoseEstimator m_PoseE;
  // private Climb m_climb;
  private Swerve m_drivetrain;
  private Vision m_vision;
  private Leds m_leds;
  private MotorConfigs m_motorConfigs = new MotorConfigs();

  private SendableChooser<Command> m_chooser;
  
  
  
  private Command m_BagIn;
  private Command m_BagOut;
  // private Command m_PivotIntakeIn;
  // private Command m_PivotIntakeOut;
  // private Command m_ElevatorUp;
  // private Command m_ElevatorDown;
  private Command m_apriltagCentering;
  private Command m_defaultElevatorCommand;
  private Command m_defaultIntakeCommand;
  private Command m_forwardMeter;
  private Command m_fieldRelativeDrive;
  private Command m_closeServo;
  private Command m_openServo;
  private Command m_pivotClimbIn;
  private Command m_pivotClimbOut;
  private Command m_LeaveZone;
  private Command m_ClimbConstantShiftUp;
  private Command m_ClimbConstantShiftDown;
  private Command m_leaveZone;
  private Command m_poorLeaveZone;
  private Command m_coralAlign;
  private Command m_defaultDrive;


  private SequentialCommandGroup m_BluePoorMansAutoLeft;
  private SequentialCommandGroup m_BluePoorMansAutoRight;
  private SequentialCommandGroup m_RedPoorMansAutoLeft;
  private SequentialCommandGroup m_RedPoorMansAutoRight;
  private SequentialCommandGroup m_twoAlgaeAuto;
  private SequentialCommandGroup m_autoL1;
  private SequentialCommandGroup m_autoL2;
  private SequentialCommandGroup m_autoL3;
  private SequentialCommandGroup m_autoBottomAlgae;
  private SequentialCommandGroup m_autoTopAlgae;
  private SequentialCommandGroup m_autoCoralStation;
  private SequentialCommandGroup m_autoCoralStation2;

  private SequentialCommandGroup m_autoZero;
  private SequentialCommandGroup m_outAndOpenClimb;
  private SequentialCommandGroup m_inAndClosedClimb;



  public RobotContainer() {
    // m_PoseE = new PoseEstimator();
    m_drivetrain = new Swerve();
    m_payload = Payload.getInstance();
    m_intake = Intake.getInstance();
    m_bag = Bag.getInstance();

    m_vision = Vision.getVisionInstance();
    m_leds = Leds.getInstance();
    
    m_chooser = new SendableChooser<>();

    m_BagIn = new BagIn(m_bag);
    m_BagOut =  new BagOut(m_bag);
    // m_forwardMeter = new MoveForward(m_drivetrain, m_poseEstimator, m_trajCreation, m_vision, 0, false);
    // m_PivotIntakeOut = new PivotIntakeOut(m_intake, m_payload);
    // m_PivotIntakeIn = new PivotIntakeIn(m_intake,m_payload);
    // m_ElevatorUp = new ElevatorUp(m_payload);
    // m_ElevatorDown = new ElevatorDown(m_payload);
    // m_LeaveZone = new LeaveZone(m_drivetrain, m_vision);
    // m_poorLeaveZone = new PoorLeaveZone(m_drivetrain, m_vision);

    m_defaultElevatorCommand = new ManualElevatorControl(m_payload);
    m_defaultIntakeCommand = new ManualIntakePivotControl(m_intake);

    m_defaultDrive = new ArcadeDrive(
      () -> -ControlMap.driver_controls.getLeftY()*Constants.DrivetrainConstants.xMultiple, 
      () -> -ControlMap.driver_controls.getLeftX()*Constants.DrivetrainConstants.yMultiple, 
      () -> -ControlMap.driver_controls.getRightX()*Constants.DrivetrainConstants.zMultiple,
      m_drivetrain);

    // m_defaultDrive = new DefaultDrive(m_drivetrain);
    // m_fieldRelativeDrive = new FieldRelativeDrive(m_drivetrain);

    // m_apriltagCentering = new ApriltagAlign(m_poseEstimator, m_vision, m_trajCreation, -0.2, -0.2);
    // m_leaveZone = new LeaveZone(m_drivetrain, m_vision);
    // m_coralAlign = new CoralStationAlign(m_poseEstimator, m_vision, m_trajCreation, Units.inchesToMeters(16), -0.40);

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
    .andThen(new AutoOutCoral(m_intake, m_bag))
    .andThen(new SetElevatorPosition(m_payload,m_intake, Constants.ElevatorConstants.basePosition));

    m_autoL2 = new SequentialCommandGroup(new SetIntakePivotPosition(m_intake, m_payload, Constants.IntakeConstants.L2Position))
    .andThen(new SetElevatorPosition(m_payload, m_intake, Constants.ElevatorConstants.L2Position));

    m_autoL3 =  new SequentialCommandGroup(new SetIntakePivotPosition(m_intake, m_payload, Constants.IntakeConstants.L3Position))
    .andThen(new SetElevatorPosition(m_payload, m_intake, Constants.ElevatorConstants.L3Position));

    m_autoBottomAlgae = new SequentialCommandGroup(new SetIntakePivotPosition(m_intake, m_payload, Constants.IntakeConstants.algaeIntakePosition))
    .andThen(new SetElevatorPosition(m_payload, m_intake, Constants.ElevatorConstants.bottomAlgaePosition));

    m_autoTopAlgae = new SequentialCommandGroup(new SetIntakePivotPosition(m_intake, m_payload, Constants.IntakeConstants.algaeIntakePosition))
    .andThen(new SetElevatorPosition(m_payload, m_intake, Constants.ElevatorConstants.topAlgaePosition));

    m_autoZero = new SequentialCommandGroup(new ZeroIntake(m_intake))
    .andThen(new ZeroElevator(m_payload));


    // m_BluePoorMansAutoLeft = new SequentialCommandGroup(
    // new PoorLeaveZone(m_drivetrain, m_vision))
    // .andThen(new AutoSpecificTag(m_poseEstimator, m_vision, m_trajCreation, -Constants.AutoConstants.xApriltagDisplacement, -Constants.AutoConstants.yApriltagDisplacementright, 22))
    // .alongWith(new ZeroIntake(m_intake).andThen((new ZeroElevator(m_payload))))
    // .andThen(new SetIntakePivotPosition(m_intake, m_payload, Constants.IntakeConstants.L2Position))
    // .andThen(new SetElevatorPosition(m_payload, m_intake, Constants.ElevatorConstants.L2Position))
    // .andThen(new SetBagSpeedTimed(m_bag))
    // .andThen(new SetIntakePivotPosition(m_intake, m_payload, Constants.IntakeConstants.algaeIntakePosition))
    // .andThen(new SetElevatorPosition(m_payload, m_intake, Constants.ElevatorConstants.topAlgaePosition))
    // .andThen(new AutoDrive(m_drivetrain, 0.2, 0, 0.5))
    // .andThen(new AutoDrive(m_drivetrain, 0, 0.2, 0.5))
    // .andThen(new AutoDrive(m_drivetrain, -0.2, 0, 0.5))
    // .andThen(new AutoDrive(m_drivetrain,0,0,0));
      

    // m_BluePoorMansAutoRight = new SequentialCommandGroup(
    //   new PoorLeaveZone(m_drivetrain, m_vision))
    //   .andThen(new AutoSpecificTag(m_poseEstimator, m_vision, m_trajCreation, -Constants.AutoConstants.xApriltagDisplacement, -Constants.AutoConstants.yApriltagDisplacementright, 20))
    //   .alongWith(new ZeroIntake(m_intake).andThen((new ZeroElevator(m_payload))))
    //   .andThen(new SetIntakePivotPosition(m_intake, m_payload, Constants.IntakeConstants.L2Position))
    //   .andThen(new SetElevatorPosition(m_payload, m_intake, Constants.ElevatorConstants.L2Position))
    //   .andThen(new SetBagSpeedTimed(m_bag))
    //   .andThen(new SetIntakePivotPosition(m_intake, m_payload, Constants.IntakeConstants.algaeIntakePosition))
    //   .andThen(new SetElevatorPosition(m_payload, m_intake, Constants.ElevatorConstants.topAlgaePosition))
    //   .andThen(new AutoDrive(m_drivetrain, 0.2, 0, 0.5))
    //   .andThen(new AutoDrive(m_drivetrain, 0, -0.2, 0.5))
    //   .andThen(new AutoDrive(m_drivetrain, -0.2, 0, 0.5))
    //   .andThen(new AutoDrive(m_drivetrain,0,0,0));


    //   m_RedPoorMansAutoLeft = new SequentialCommandGroup(
    //     new PoorLeaveZone(m_drivetrain, m_vision))
    //     .andThen(new AutoSpecificTag(m_poseEstimator, m_vision, m_trajCreation, -Constants.AutoConstants.xApriltagDisplacement, -Constants.AutoConstants.yApriltagDisplacementright, 9))
    //     .alongWith(new ZeroIntake(m_intake).andThen((new ZeroElevator(m_payload))))
    //     .andThen(new SetIntakePivotPosition(m_intake, m_payload, Constants.IntakeConstants.L2Position))
    //     .andThen(new SetElevatorPosition(m_payload, m_intake, Constants.ElevatorConstants.L2Position))
    //     .andThen(new SetBagSpeedTimed(m_bag))
    //     .andThen(new SetIntakePivotPosition(m_intake, m_payload, Constants.IntakeConstants.algaeIntakePosition))
    //     .andThen(new SetElevatorPosition(m_payload, m_intake, Constants.ElevatorConstants.topAlgaePosition))
    //     .andThen(new AutoDrive(m_drivetrain, 0.2, 0, 0.5))
    //     .andThen(new AutoDrive(m_drivetrain, 0, 0.2, 0.5))
    //     .andThen(new AutoDrive(m_drivetrain, -0.2, 0, 0.5))
    //     .andThen(new AutoDrive(m_drivetrain,0,0,0));

    //   m_RedPoorMansAutoRight = new SequentialCommandGroup(
    //     new PoorLeaveZone(m_drivetrain, m_vision))
    //     .andThen(new AutoSpecificTag(m_poseEstimator, m_vision, m_trajCreation, -Constants.AutoConstants.xApriltagDisplacement, -Constants.AutoConstants.yApriltagDisplacementright, 11))
    //     .alongWith(new ZeroIntake(m_intake).andThen((new ZeroElevator(m_payload))))
    //     .andThen(new SetIntakePivotPosition(m_intake, m_payload, Constants.IntakeConstants.L2Position))
    //     .andThen(new SetElevatorPosition(m_payload, m_intake, Constants.ElevatorConstants.L2Position))
    //     .andThen(new SetBagSpeedTimed(m_bag))
    //     .andThen(new SetIntakePivotPosition(m_intake, m_payload, Constants.IntakeConstants.algaeIntakePosition))
    //     .andThen(new SetElevatorPosition(m_payload, m_intake, Constants.ElevatorConstants.topAlgaePosition))
    //     .andThen(new AutoDrive(m_drivetrain, 0.2, 0, 0.5))
    //     .andThen(new AutoDrive(m_drivetrain, 0, -0.2, 0.5))
    //     .andThen(new AutoDrive(m_drivetrain, -0.2, 0, 0.5))
    //     .andThen(new AutoDrive(m_drivetrain,0,0,0));

    //     m_twoAlgaeAuto = new SequentialCommandGroup(
    //       new PoorLeaveZone(m_drivetrain, m_vision))
    //       .andThen(new AutoSpecificTag(m_poseEstimator, m_vision, m_trajCreation, -Constants.AutoConstants.xApriltagDisplacement, -Constants.AutoConstants.yApriltagDisplacementright, 20))
    //       .alongWith(new ZeroIntake(m_intake).andThen((new ZeroElevator(m_payload))))
    //       .andThen(new SetIntakePivotPosition(m_intake, m_payload, Constants.IntakeConstants.L2Position))
    //       .andThen(new SetElevatorPosition(m_payload, m_intake, Constants.ElevatorConstants.L2Position))
    //       .andThen(new SetBagSpeedTimed(m_bag))
    //       .andThen(new SetIntakePivotPosition(m_intake, m_payload, Constants.IntakeConstants.algaeIntakePosition))
    //       .andThen(new SetElevatorPosition(m_payload, m_intake, Constants.ElevatorConstants.topAlgaePosition))
    //       .andThen(new AutoDrive(m_drivetrain, 0.2, 0, 0.5))
    //       .andThen(new AutoDrive(m_drivetrain, 0, 0.2, 0.5))
    //       .andThen(new AutoDrive(m_drivetrain, -0.2, 0, 0.5))
    //       .andThen(new AutoDrive(m_drivetrain,0,0,0))
    //       .andThen(new AutoDrive(m_drivetrain, -0.5,0,1))
    //       .andThen(new AutoSpecificTag(m_poseEstimator, m_vision, m_trajCreation, -Constants.AutoConstants.xApriltagDisplacement, -Constants.AutoConstants.yApriltagDisplacementright, 19))
    //       .andThen(new SetIntakePivotPosition(m_intake, m_payload, Constants.IntakeConstants.algaeIntakePosition))
    //       .andThen(new SetElevatorPosition(m_payload, m_intake, Constants.ElevatorConstants.bottomAlgaePosition))
    //       .andThen(new AutoDrive(m_drivetrain, 0.2, 0, 0.5))
    //       .andThen(new AutoDrive(m_drivetrain, 0, 0.2, 0.5))
    //       .andThen(new AutoDrive(m_drivetrain, -0.2, 0, 0.5))
    //       .andThen(new AutoDrive(m_drivetrain,0,0,0));

    m_outAndOpenClimb = null; //new SequentialCommandGroup(m_pivotClimbOut).andThen(m_openServo);
    m_inAndClosedClimb = null;//new SequentialCommandGroup(m_closeServo).andThen(m_pivotClimbIn);

    NamedCommands.registerCommand("autoL1", m_autoL1);
    NamedCommands.registerCommand("autoL2", m_autoL2);
    NamedCommands.registerCommand("autoL3", m_autoL3);
    // NamedCommands.registerCommand("AprilCenter", m_apriltagCentering);
    NamedCommands.registerCommand("autoCoral", m_autoCoralStation2);

    m_chooser.addOption("Auto L1", m_autoL1);
    m_chooser.addOption("Auto L2", m_autoL2);
    m_chooser.addOption("Auto L3", m_autoL3);
    // m_chooser.addOption("Forward Meter",m_forwardMeter);
    // m_chooser.addOption("Leave Zone",m_LeaveZone);

    m_chooser.setDefaultOption("Nothing Selected", null);
    // m_chooser.addOption("BLUE Poor Man's Auto Right", m_BluePoorMansAutoLeft);
    // m_chooser.addOption("BLUE Poor Man's Auto Left", m_BluePoorMansAutoRight);
    // m_chooser.addOption("RED Poor Man's Auto Right", m_RedPoorMansAutoLeft);
    // m_chooser.addOption("RED Poor Man's Auto Left", m_RedPoorMansAutoRight);
    // m_chooser.addOption("(DO NOT RUN) Two Algae Removal", m_twoAlgaeAuto);

    SmartDashboard.putData(m_chooser);
  }



  private void configureBindings() {
    ControlMap.driver_controls.leftBumper().onTrue(new InstantCommand(() -> m_drivetrain.resetGyro()));
    // ControlMap.driver_controls.leftTrigger().onTrue(new ApriltagAlign(m_poseEstimator, m_vision, m_trajCreation, 
    // -Constants.AutoConstants.xApriltagDisplacement,
    // Constants.AutoConstants.yApriltagDisplacementleft));
    // ControlMap.driver_controls.rightTrigger().onTrue(new ApriltagAlign(m_poseEstimator, m_vision, m_trajCreation, 
    // -Constants.AutoConstants.xApriltagDisplacement,
    // -Constants.AutoConstants.yApriltagDisplacementright));

    ControlMap.gunnerButton1.whileTrue(m_BagIn);
    ControlMap.gunnerButton2.whileTrue(m_BagOut);
    ControlMap.gunnerButton3.onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll())); //
    ControlMap.gunnerButton8.onTrue(m_autoBottomAlgae);
    ControlMap.gunnerButton9.onTrue(m_autoTopAlgae);
    ControlMap.gunnerButton14.onTrue(m_autoZero);
    ControlMap.gunnerButton13.onTrue(m_autoL2);
    ControlMap.gunnerButton12.onTrue(m_autoL3);
    ControlMap.gunnerButton11.onTrue(m_autoCoralStation);
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