// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.DriverCommands;

import frc.robot.subsystems.PoseEstimator;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision;
import frc.robot.util.TrajectoryCreation;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoSpecificTag extends Command {
  private PoseEstimator m_poseEstimator;
  private Vision m_vision;
  private TrajectoryCreation m_traj;
  private double displacementX;
  private double displacementY;
  private int m_id;
  private Command controllerCommand;
  /** Creates a new ApriltagAlign. */
  public AutoSpecificTag(PoseEstimator p, Vision v, TrajectoryCreation t, double disX, double disY, int id) {
    m_poseEstimator = p;
    m_vision = v;
    m_traj = t;
    displacementX = disX;
    displacementY = disY;
    m_id = id;
    addRequirements(m_poseEstimator);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    PathPlannerPath path = null;
    try {
      // if (DriverStation.getAlliance().get() == Alliance.Red){
      //   if (m_id == 22){
      //     m_id = 9;
      //   }
      //   else {
      //     m_id = 11;
      //   }
      // }
      path = m_traj.autoCentering(m_poseEstimator, m_vision, displacementX, displacementY, m_id);
      path.preventFlipping = true;
      controllerCommand = AutoBuilder.followPath(path);
    }
    catch(Exception e) {
      System.out.println("Path could not be generated");

      controllerCommand = new Command() {
        @Override
        public boolean isFinished() {
            return true;
        }
      };
    }

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
