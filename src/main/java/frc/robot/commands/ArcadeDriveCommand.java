// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.mySwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArcadeDriveCommand extends Command {
  /** Creates a new ArcadeDriveCommand. */

  Supplier<Double> x;
  Supplier<Double> y;
  Supplier<Double> zRot;
  mySwerveSubsystem m_subsystem;
  SwerveDriveKinematics kinematics = Constants.swerveKinematics;


  public ArcadeDriveCommand(Supplier<Double> xSpeed, Supplier<Double> ySpeed, Supplier<Double> zRotation, mySwerveSubsystem subsystem) {
    x = xSpeed;
    y = ySpeed;
    zRot = zRotation;
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setRobotBassedOffFieldChassisSpeeds(new ChassisSpeeds());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = x.get();
    double ySpeed = y.get();
    double zRotation = zRot.get();
    if (Math.abs(xSpeed) < Constants.Deadband) xSpeed = 0;
    if (Math.abs(ySpeed) < Constants.Deadband) ySpeed = 0;
    if (Math.abs(zRotation) < Constants.Deadband) zRotation = 0;
    ChassisSpeeds speeds = ChassisSpeeds.fromRobotRelativeSpeeds(xSpeed,ySpeed,zRotation,m_subsystem.getHeading());
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.MAX_SPEED);
    m_subsystem.setAllModuleStates(states);
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
