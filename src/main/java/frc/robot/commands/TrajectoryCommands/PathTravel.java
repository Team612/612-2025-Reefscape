// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TrajectoryCommands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Vision;

public class PathTravel extends Command {
  private final Swerve driveSystem;
  private final Vision m_vision;
  private final PoseEstimator poseEstimatorSystem;
  private final TrajectoryCreation m_traj;
  private final double translation;
  private final String dire;

  private Command controllerCommand = Commands.none();

  /** Creates a new RunOnTheFly. */
  public PathTravel(Swerve d, PoseEstimator p, TrajectoryCreation m_trajectoryCreation, Vision v, 
                    double y, String dir) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveSystem = d;
    poseEstimatorSystem = p;
    m_traj = m_trajectoryCreation;
    m_vision = v;
    translation = y;
    dire = dir;


    addRequirements(d, v, p);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    PathPlannerPath path = null;
    if (dire.equals("f")){
      path = m_traj.Forward(poseEstimatorSystem, translation);
    }
    if (dire.equals("b")){
      path = m_traj.Backward(poseEstimatorSystem, translation);
    }
    if (dire.equals("l")){
      path = m_traj.StrafeLeft(poseEstimatorSystem, translation);
    }
    if (dire.equals("r")){
      path = m_traj.StrafeRight(poseEstimatorSystem, translation);
    }
  
  
    controllerCommand = AutoBuilder.followPath(path);
    controllerCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    controllerCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    controllerCommand.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controllerCommand.isFinished();
  }
}