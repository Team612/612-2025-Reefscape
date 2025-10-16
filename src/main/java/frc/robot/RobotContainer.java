package frc.robot;

import frc.robot.Constants.BagConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PayloadConstants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.ArcadeGun;
import frc.robot.commands.AutoBagOut;
import frc.robot.commands.DriverStationArcadeGun;
import frc.robot.commands.DriverStationBag;
import frc.robot.commands.ManualBag;
import frc.robot.commands.PayloadSetPoint;
import frc.robot.commands.SetOdometryPosition;
import frc.robot.commands.SetPayloadPosition;
import frc.robot.commands.SimpleLeaveZone;
import frc.robot.commands.Zero;
import frc.robot.subsystems.Bag;
import frc.robot.subsystems.Payload;
import frc.robot.subsystems.Swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  private final Swerve m_swerve = new Swerve();
  private final Payload m_payload = new Payload();
  private final Bag m_bag = new Bag();

  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  // SWITCH WITH CONTROLLERS
  private final CommandXboxController m_gunController = new CommandXboxController(OperatorConstants.kGunnerControllerPort);
  // private static Joystick gunner_controls = new Joystick(OperatorConstants.kGunnerDriverStationPort1);
  // private static Joystick gunner_controls_2 = new Joystick(OperatorConstants.kGunnerDriverStationPort2);
  // private static JoystickButton gunnerButton1 = new JoystickButton(gunner_controls, 11);
  // private static JoystickButton gunnerButton2 = new JoystickButton(gunner_controls, 9);
  // private static JoystickButton gunnerButton8 = new JoystickButton(gunner_controls, 5);
  // private static JoystickButton gunnerButton9 = new JoystickButton(gunner_controls, 7);
  // private static JoystickButton gunnerButton11 = new JoystickButton(gunner_controls, 3);
  // private static JoystickButton gunnerButton12 = new JoystickButton(gunner_controls, 10);
  // private static JoystickButton gunnerButton13 = new JoystickButton(gunner_controls, 6);
  // private static JoystickButton gunnerButton14 = new JoystickButton(gunner_controls_2, 12);

  private SendableChooser<Command> autoChooser;

  public RobotContainer() {
    NamedCommands.registerCommand("L2",new PayloadSetPoint(m_payload, PayloadConstants.L2Position, PayloadConstants.coralScoringPosition));
    NamedCommands.registerCommand("BottomAlgie",new PayloadSetPoint(m_payload, PayloadConstants.bottomAlgaePosition, PayloadConstants.algaeIntakePosition));
    NamedCommands.registerCommand("TopAlgie",new PayloadSetPoint(m_payload, PayloadConstants.topAlgaePosition, PayloadConstants.algaeIntakePosition));
    NamedCommands.registerCommand("BagOut",new AutoBagOut(m_bag));
    NamedCommands.registerCommand("SimpleLeaveZone", new SimpleLeaveZone(m_swerve));

    m_swerve.setDefaultCommand(new ArcadeDrive(m_swerve,m_driverController));

    // SWITCH WITH CONTROLLERS
    m_payload.setDefaultCommand(new ArcadeGun(m_payload,m_gunController));
    m_bag.setDefaultCommand(new ManualBag(m_bag,m_gunController));
    // m_payload.setDefaultCommand(new DriverStationArcadeGun(m_payload,gunner_controls));
    
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    m_driverController.leftBumper().onTrue(new SetOdometryPosition(m_swerve, new Pose2d()));
    m_driverController.rightBumper().onTrue(new SetOdometryPosition(m_swerve, new Pose2d(0,0,new Rotation2d(Math.PI))));
    
    // SWITCH WITH CONTROLLERS
    Trigger manualOverrideTrigger = new Trigger(() ->
      Math.abs(m_gunController.getLeftY()) > PayloadConstants.initializeDEADBAND
      || Math.abs(m_gunController.getRightX()) > PayloadConstants.initializeDEADBAND
    );
    manualOverrideTrigger.onTrue(new ArcadeGun(m_payload,m_gunController));
    m_gunController.x().onTrue(new Zero(m_payload));
    m_gunController.a().onTrue(new PayloadSetPoint(m_payload, PayloadConstants.L2Position, PayloadConstants.coralScoringPosition));
    m_gunController.y().onTrue(new PayloadSetPoint(m_payload, PayloadConstants.L3Position, PayloadConstants.coralScoringPosition));
    m_gunController.b().onTrue(new PayloadSetPoint(m_payload, PayloadConstants.CoralStationElevatorPosition, PayloadConstants.CoralStationIntakePosition));
    m_gunController.leftBumper().onTrue(new PayloadSetPoint(m_payload, PayloadConstants.bottomAlgaePosition, PayloadConstants.algaeIntakePosition));
    m_gunController.rightBumper().onTrue(new PayloadSetPoint(m_payload, PayloadConstants.topAlgaePosition, PayloadConstants.algaeIntakePosition));
    // Trigger driverStationManualOverrideTrigger = new Trigger(() ->
    //   (gunner_controls.getRawAxis(0) != -0.0078125)
    //   || (gunner_controls.getRawAxis(1) != -0.0078125)
    // );
    // driverStationManualOverrideTrigger.onTrue(new DriverStationArcadeGun(m_payload,gunner_controls));
    // gunnerButton1.whileTrue(new DriverStationBag(m_bag, BagConstants.driverBagSpeed));
    // gunnerButton2.whileTrue(new DriverStationBag(m_bag, -BagConstants.driverBagSpeed));
    // gunnerButton8.onTrue(new PayloadSetPoint(m_payload, PayloadConstants.bottomAlgaePosition, PayloadConstants.algaeIntakePosition));
    // gunnerButton9.onTrue(new PayloadSetPoint(m_payload, PayloadConstants.topAlgaePosition, PayloadConstants.algaeIntakePosition));
    // gunnerButton14.onTrue(new Zero(m_payload));
    // gunnerButton13.onTrue(new PayloadSetPoint(m_payload, PayloadConstants.L2Position, PayloadConstants.coralScoringPosition));
    // gunnerButton12.onTrue(new PayloadSetPoint(m_payload, PayloadConstants.L3Position, PayloadConstants.coralScoringPosition));
    // gunnerButton11.onTrue(new PayloadSetPoint(m_payload, PayloadConstants.CoralStationElevatorPosition, PayloadConstants.CoralStationIntakePosition));
  }

  public Command getAutonomousCommand() {
    // return new SequentialCommandGroup(
    //   new SetOdometryPosition(m_swerve, new Pose2d(0,0,new Rotation2d(Math.PI))),
    //   new SetPayloadPosition(m_payload, PayloadConstants.startingElevatorPos, PayloadConstants.startingPivotPos),
    //   autoChooser.getSelected(),
    //   new AutoBagOut(m_bag)
    // );

    // all auto commands
    // return new AutoBagOut(m_bag)
    // return new PayloadSetPoint(m_payload, PayloadConstants.L2Position, PayloadConstants.coralScoringPosition)
    // return new PayloadSetPoint(m_payload, PayloadConstants.topAlgaePosition, PayloadConstants.algaeIntakePosition)
    // return new PayloadSetPoint(m_payload, PayloadConstants.bottomAlgaePosition, PayloadConstants.algaeIntakePosition)
    // return new SimpleLeaveZone(m_swerve)

    // only use if pathplanner is absolutely refusing to run properly
    return new SequentialCommandGroup(
      new SetOdometryPosition(m_swerve, new Pose2d(0,0,new Rotation2d(Math.PI))),
      new SetPayloadPosition(m_payload, PayloadConstants.startingElevatorPos, PayloadConstants.startingPivotPos),
      new SimpleLeaveZone(m_swerve)
    );
  }
}
