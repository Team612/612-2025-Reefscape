// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.util.ControlMap;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DefaultDrive extends Command {
  /** Creates a new ArcadeDriveCommand. */

  Swerve m_drivetrain;
  SwerveDriveKinematics kinematics = Constants.SwerveConstants.swerveKinematics;


  public DefaultDrive(Swerve subsystem) {
    m_drivetrain = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_subsystem.setRobotBassedOffFieldChassisSpeeds(new ChassisSpeeds());
    m_drivetrain.setModuleState(new SwerveModuleState[4]);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = -ControlMap.driver_controls.getLeftY()*Constants.SwerveConstants.xMultiple;
    double ySpeed = -ControlMap.driver_controls.getLeftX()*Constants.SwerveConstants.yMultiple;
    double zRotation = -ControlMap.driver_controls.getRightX()*Constants.SwerveConstants.zMultiple;
    if (Math.abs(xSpeed) < Constants.SwerveConstants.Deadband) xSpeed = 0;
    if (Math.abs(ySpeed) < Constants.SwerveConstants.Deadband) ySpeed = 0;
    if (Math.abs(zRotation) < Constants.SwerveConstants.Deadband) zRotation = 0;
    
    ChassisSpeeds speeds = ChassisSpeeds.fromRobotRelativeSpeeds(xSpeed,ySpeed,zRotation,m_drivetrain.getHeading());
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.SwerveConstants.MAX_SPEED);

    m_drivetrain.setModuleState(states);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
