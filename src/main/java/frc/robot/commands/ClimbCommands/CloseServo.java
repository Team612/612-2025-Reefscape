// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.ClimbCommands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.subsystems.Climb;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class CloseServo extends Command {
//   /** Creates a new ArmsCommand. */
//   private Climb m_climb;
//   public CloseServo(Climb c) {
//     m_climb = c;
//     addRequirements(m_climb);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     System.out.println(Constants.ClimbConstants.servoClosePosition);
//     m_climb.setClawPosition(Constants.ClimbConstants.servoClosePosition);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return (m_climb.getServoPosition() <= Constants.ClimbConstants.servoClosePosition);
//   }
// }