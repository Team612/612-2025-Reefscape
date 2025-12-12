package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.ResetEncoders;
import frc.robot.subsystems.Swerve;

public class RobotContainer {

  private Swerve m_swerve = new Swerve();

  private CommandXboxController controller = new CommandXboxController(Constants.controllerPortNumber);

  private SendableChooser<Command> autoChooser;

  public RobotContainer() {
    m_swerve.setDefaultCommand(new ArcadeDrive(m_swerve, controller));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    controller.leftBumper().onTrue(new ResetEncoders(m_swerve));
  }

  public Command getAutonomousCommand() {
    return AutoBuilder.followPath(m_swerve.Forward());
    // return autoChooser.getSelected();
  }
}
