// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Constants {
    public static final int elevatorID = 1;
    public static final int elevatorID2 = 2;
    public static double payspeed = 0;
    public static double kPositionConversionFactor;
    public static final int toplimitSwitchID = 0;
    public static final int bottomlimitSwitchID = 0;
    public final class ElevatorConstants {
        public static final double kP = 0.1;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
      }
  
      public final class ElevatorControllerVelocityGains {
        public static final double kP = 0.0001;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
    }
    public final class ClimbConstants {
        public static final double kP = 0.1;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
      }
  
      public final class ClimbControllerVelocityGains {
        public static final double kP = 0.0001;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
    }
    // public static final int neoPivotID = 0;
}
