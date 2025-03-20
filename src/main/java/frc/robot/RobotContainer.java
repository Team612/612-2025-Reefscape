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
// import frc.robot.util.TrajectoryConfiguration;
// import frc.robot.util.TrajectoryCreation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoCommands.DriverCommands.ApriltagAlign;
import frc.robot.commands.AutoCommands.DriverCommands.AutoDrive;
import frc.robot.commands.AutoCommands.DriverCommands.AutoSpecificTag;
import frc.robot.commands.AutoCommands.DriverCommands.CoralStationAlign;
import frc.robot.commands.AutoCommands.DriverCommands.LeaveZone;
import frc.robot.commands.AutoCommands.DriverCommands.PoorLeaveZone;
import frc.robot.commands.AutoCommands.GunnerCommands.SetBagSpeedInTimed;
// import frc.robot.commands.AutoCommands.DriverCommands.ApriltagAlign;
import frc.robot.commands.AutoCommands.GunnerCommands.SetBagSpeedTimed;
// import frc.robot.commands.ClimbCommands.CloseServo;
// import frc.robot.commands.ClimbCommands.ManualClimbPivotControls;
// import frc.robot.commands.ClimbCommands.OpenServo;
// import frc.robot.commands.ClimbCommands.PivotClimbIn;
// import frc.robot.commands.ClimbCommands.PivotClimbOut;
import frc.robot.commands.DriveCommands.DefaultDrive;
import frc.robot.commands.DriveCommands.FeedForwardCharacterization;
import frc.robot.commands.DriveCommands.FeedForwardCharacterization.FeedForwardCharacterizationData;
import frc.robot.commands.DriveCommands.FieldRelativeDrive;
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
// import frc.robot.commands.AutoCommands.DriverCommands.MoveForward;
// import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Mecanum;
import frc.robot.subsystems.Payload;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Vision;
// import frc.robot.subsystems.Vision;
// import frc.robot.commands.ClimbCommands.ClimbConstantShift;
import frc.robot.util.ControlMap;
import frc.robot.util.MotorConfigs;
import frc.robot.util.PathPlannerUtil;
import frc.robot.util.TrajectoryConfiguration;
import frc.robot.util.TrajectoryCreation;

public class RobotContainer {
  private Payload m_payload;
  private Intake m_intake;
  private Bag m_bag; 
  // private Climb m_climb;
  private Mecanum m_drivetrain;
  private PoseEstimator m_poseEstimator;
  private Vision m_vision;
  private Leds m_leds;
  private TrajectoryConfiguration m_trajConfigs;
  private TrajectoryCreation m_trajCreation;
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
  private Command m_defaultDrive;
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


  private SequentialCommandGroup m_poorMansAutoLeft;
  private SequentialCommandGroup m_poorMansAutoRight;
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
    m_payload = Payload.getInstance();
    m_intake = Intake.getInstance();
    m_drivetrain = Mecanum.getInstance();
    m_bag = Bag.getInstance();

    m_poseEstimator = PoseEstimator.getPoseEstimatorInstance();
    m_vision = Vision.getVisionInstance();
    m_trajConfigs = TrajectoryConfiguration.getInstance();
    m_trajCreation = TrajectoryCreation.getInstance();
    m_leds = Leds.getInstance();
    
    m_chooser = new SendableChooser<>();

    m_BagIn = new BagIn(m_bag);
    m_BagOut =  new BagOut(m_bag);
    // m_forwardMeter = new MoveForward(m_drivetrain, m_poseEstimator, m_trajCreation, m_vision, 0, false);
    // m_PivotIntakeOut = new PivotIntakeOut(m_intake, m_payload);
    // m_PivotIntakeIn = new PivotIntakeIn(m_intake,m_payload);
    // m_ElevatorUp = new ElevatorUp(m_payload);
    // m_ElevatorDown = new ElevatorDown(m_payload);
    m_LeaveZone = new LeaveZone(m_drivetrain, m_vision);
    m_poorLeaveZone = new PoorLeaveZone(m_drivetrain, m_vision);

    m_defaultElevatorCommand = new ManualElevatorControl(m_payload);
    m_defaultIntakeCommand = new ManualIntakePivotControl(m_intake);

    m_defaultDrive = new DefaultDrive(m_drivetrain);
    m_fieldRelativeDrive = new FieldRelativeDrive(m_drivetrain);

    m_apriltagCentering = new ApriltagAlign(m_poseEstimator, m_vision, m_trajCreation, -0.2, -0.2);
    m_leaveZone = new LeaveZone(m_drivetrain, m_vision);
    m_coralAlign = new CoralStationAlign(m_poseEstimator, m_vision, m_trajCreation, Units.inchesToMeters(16), -0.40);

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

    // m_autoCoralStation = new SequentialCommandGroup(m_payload.profiledElevatorCommand(Constants.ElevatorConstants.CoralStationPosition)
    // .andThen(new SetIntakePivotPosition(m_intake, m_payload, Constants.IntakeConstants.CoralStationPosition)));

    // m_autoCoralStation2 = new SequentialCommandGroup(new SetElevatorPosition(m_payload, m_intake,Constants.ElevatorConstants.CoralStationPosition)
    // .andThen(new SetIntakePivotPosition(m_intake, m_payload, Constants.IntakeConstants.CoralStationPosition))
    // .andThen(new SetBagSpeedInTimed(m_intake)));

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


    m_poorMansAutoLeft = new SequentialCommandGroup(
    new PoorLeaveZone(m_drivetrain, m_vision))
    .andThen(new AutoSpecificTag(m_poseEstimator, m_vision, m_trajCreation, -Constants.AutoConstants.xApriltagDisplacement, -Constants.AutoConstants.yApriltagDisplacementright, 22))
    .alongWith(new ZeroIntake(m_intake).andThen((new ZeroElevator(m_payload))))
    .andThen(new SetIntakePivotPosition(m_intake, m_payload, Constants.IntakeConstants.L2Position))
    .andThen(new SetElevatorPosition(m_payload, m_intake, Constants.ElevatorConstants.L2Position))
    .andThen(new SetBagSpeedTimed(m_bag))
    .andThen(new SetIntakePivotPosition(m_intake, m_payload, Constants.IntakeConstants.algaeIntakePosition))
    .andThen(new SetElevatorPosition(m_payload, m_intake, Constants.ElevatorConstants.topAlgaePosition))
    .andThen(new AutoDrive(m_drivetrain, 0.2, 0, 0.5))
    .andThen(new AutoDrive(m_drivetrain, 0, 0.2, 0.5))
    .andThen(new AutoDrive(m_drivetrain, -0.2, 0, 0.5))
    .andThen(new AutoDrive(m_drivetrain,0,0,0));
      

    m_poorMansAutoRight = new SequentialCommandGroup(
      new PoorLeaveZone(m_drivetrain, m_vision))
      .andThen(new AutoSpecificTag(m_poseEstimator, m_vision, m_trajCreation, -Constants.AutoConstants.xApriltagDisplacement, -Constants.AutoConstants.yApriltagDisplacementright, 20))
      .alongWith(new ZeroIntake(m_intake).andThen((new ZeroElevator(m_payload))))
      .andThen(new SetIntakePivotPosition(m_intake, m_payload, Constants.IntakeConstants.L2Position))
      .andThen(new SetElevatorPosition(m_payload, m_intake, Constants.ElevatorConstants.L2Position))
      .andThen(new SetBagSpeedTimed(m_bag))
      .andThen(new SetIntakePivotPosition(m_intake, m_payload, Constants.IntakeConstants.algaeIntakePosition))
      .andThen(new SetElevatorPosition(m_payload, m_intake, Constants.ElevatorConstants.topAlgaePosition))
      .andThen(new AutoDrive(m_drivetrain, 0.2, 0, 0.5))
      .andThen(new AutoDrive(m_drivetrain, 0, -0.2, 0.5))
      .andThen(new AutoDrive(m_drivetrain, -0.2, 0, 0.5))
      .andThen(new AutoDrive(m_drivetrain,0,0,0));
        
  




    

    // m_poorMansAutoRight = new SequentialCommandGroup(new ZeroIntake(m_intake))
    // .andThen(new ZeroElevator(m_payload))
    // .andThen(new PoorLeaveZone(m_drivetrain, m_vision))
    // .andThen(new AutoSpecificTag(m_poseEstimator, m_vision, m_trajCreation, -Constants.AutoConstants.xApriltagDisplacement, -Constants.AutoConstants.yApriltagDisplacementright, 20))
    // .andThen(new SetIntakePivotPosition(m_intake, m_payload, Constants.IntakeConstants.L2Position))
    // .andThen(new SetElevatorPosition(m_payload, m_intake, Constants.ElevatorConstants.L2Position))
    // .andThen(new SetBagSpeedTimed(m_bag));

    m_outAndOpenClimb = null; //new SequentialCommandGroup(m_pivotClimbOut).andThen(m_openServo);
    m_inAndClosedClimb = null;//new SequentialCommandGroup(m_closeServo).andThen(m_pivotClimbIn);

    NamedCommands.registerCommand("autoL1", m_autoL1);
    NamedCommands.registerCommand("autoL2", m_autoL2);
    NamedCommands.registerCommand("autoL3", m_autoL3);
    NamedCommands.registerCommand("AprilCenter", m_apriltagCentering);
    NamedCommands.registerCommand("autoCoral", m_autoCoralStation2);

    m_chooser.addOption("Auto L1", m_autoL1);
    m_chooser.addOption("Auto L2", m_autoL2);
    m_chooser.addOption("Auto L3", m_autoL3);
    m_chooser.addOption("Forward Meter",m_forwardMeter);
    m_chooser.addOption("Leave Zone",m_LeaveZone);

    m_chooser.addOption("Mecanum Characterization", new FeedForwardCharacterization(
                  m_drivetrain,
                  true,
                  new FeedForwardCharacterizationData("drive"),
                  m_drivetrain::runCharacterizationVolts,
                  m_drivetrain::getCharacterizationVelocity));

    m_chooser.setDefaultOption("Nothing Selected", null);
    m_chooser.addOption("Poor Man's Auto Left", m_poorMansAutoLeft);
    m_chooser.addOption("Poor Man's Auto Right", m_poorMansAutoRight);

     List<String> autos = PathPlannerUtil.getExistingPaths();
    for (String auto : autos) {
      m_chooser.addOption(auto, AutoBuilder.buildAuto(auto));
    }

    SmartDashboard.putData(m_chooser);
  }



  private void configureBindings() {
    ControlMap.driver_controls.leftBumper().onTrue(new InstantCommand(() -> m_drivetrain.zeroGyro()));
    ControlMap.driver_controls.leftTrigger().onTrue(new ApriltagAlign(m_poseEstimator, m_vision, m_trajCreation, 
    -Constants.AutoConstants.xApriltagDisplacement,
    Constants.AutoConstants.yApriltagDisplacementleft));
    ControlMap.driver_controls.rightTrigger().onTrue(new ApriltagAlign(m_poseEstimator, m_vision, m_trajCreation, 
    -Constants.AutoConstants.xApriltagDisplacement,
    -Constants.AutoConstants.yApriltagDisplacementright));
    // ControlMap.driver_controls.a().onTrue(m_coralAlign);

    // ControlMap.driver_controls.b().onTrue(m_apriltagCentering);
    // ControlMap.driver_controls.x().onTrue(m_alignLeft);
    // ControlMap.driver_controls.a().whileTrue(m_simpleAlign);
   
    // ControlMap.driver_controls.b().onTrue(m_visio);
    // ControlMap.driver_controls.a().onTrue(m_autoL2);
    // ControlMap.driver_controls.x().onTrue(m_autoL3);

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
    ControlMap.gunnerButton2.whileTrue(m_BagOut);
    ControlMap.gunnerButton3.onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));
    ControlMap.gunnerButton4.onTrue(new SetIntakePivotPosition(m_intake,m_payload,Constants.IntakeConstants.L1Position));
    ControlMap.gunnerButton5.onTrue(new SetIntakePivotPosition(m_intake,m_payload,Constants.IntakeConstants.L2Position));
    ControlMap.gunnerButton6.onTrue(new SetIntakePivotPosition(m_intake, m_payload, Constants.IntakeConstants.CoralStationPosition));
    ControlMap.gunnerButton7.onTrue(new SetElevatorPosition(m_payload,m_intake, Constants.ElevatorConstants.L1Position));
    ControlMap.gunnerButton8.onTrue(m_autoBottomAlgae);
    ControlMap.gunnerButton9.onTrue(m_autoTopAlgae);
    ControlMap.gunnerButton10.onTrue(new SetElevatorPosition(m_payload,m_intake, Constants.ElevatorConstants.CoralStationPosition));
    ControlMap.gunnerButton14.onTrue(m_autoZero);
    ControlMap.gunnerButton13.onTrue(m_autoL2);
    ControlMap.gunnerButton12.onTrue(m_autoL3);
    ControlMap.gunnerButton11.onTrue(m_autoCoralStation);
    // ControlMap.gunnerButton12.onTrue(new SetElevatorPosition(m_payload,m_intake, Constants.ElevatorConstants.CoralStationPosition));
    // ControlMap.gunnerButton4.onTrue(new SetIntakePivotPosition(m_intake, m_payload, Constants.IntakeConstants.L1Position));

    
    // ControlMap.gunnerButton2.onTrue(new AutoOutCoral(m_intake));

    // ControlMap.gunnerButton7.onTrue(new SetElevatorPosition(m_payload, Constants.ElevatorConstants.CoralStationPosition));
    // ControlMap.gunnerButton8.onTrue(new SetElevatorPosition(m_payload, Constants.ElevatorConstants.L1Position));

    // ControlMap.gunnerButton11.onTrue(m_autoCoralStation);
   // ControlMap.gunnerButton12.onTrue(m_autoL1);


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
    m_drivetrain.setDefaultCommand(m_fieldRelativeDrive);
    m_payload.setDefaultCommand(m_defaultElevatorCommand);
    m_intake.setDefaultCommand(m_defaultIntakeCommand);
  }

  public Command getAutonomousCommand() {
    return (m_chooser.getSelected() != null) ? m_chooser.getSelected() : Commands.print("No autonomous command configured");
  }
}