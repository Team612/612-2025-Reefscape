// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.ClimbCommands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.Climb;
// import frc.robot.util.ControlMap;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class ManualClimbPivotControls extends Command {
//   /** Creates a new PivotIn. */
//   Climb m_climb;
//   double speed;
//   public ManualClimbPivotControls(Climb c, double s) {
//     m_climb = c;
//     speed = s;
//     addRequirements(m_climb);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     m_climb.setPivotSpeed(0);
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     System.out.println(speed);
//     m_climb.setPivotSpeed(speed);
    
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     m_climb.setPivotSpeed(0);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
