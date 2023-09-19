// Archivo donde se guardan las constantes del robot
////////////////////////////////////////////////////
// File where the robot constants are stored

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class Constants {

  public final static class Swerve {

    public static final class Module {
      public static final double kWheelDiameterMeters = 0.1016; // 4 inches
      public static final double kDriveMotorGearRatio = 1.0 / 6.12; // 6.12:1 Drive
      public static final double kTurningMotorGearRatio = 1.0 / 12.8; // 12.8:1 Steering

      public static final double kDriveEncoder_RotationToMeter = kDriveMotorGearRatio * kWheelDiameterMeters * Math.PI; // Conversion Rotaciones a Metros
      public static final double kDriveEncoder_RPMToMeterPerSecond = kDriveEncoder_RotationToMeter / 60.0; // Conversion RPM a Metros por Segundo

      public static final double kTurningEncoder_RotationToRadian = kTurningMotorGearRatio * 2.0 * Math.PI; // Conversion Rotaciones a Radianes
      public static final double kTurningEncoder_RPMToRadianPerSecond = kTurningEncoder_RotationToRadian / 60.0; // Conversion RPM a Radianes por Segundo
    }

    public final static class Physical {
      public static final double kTrackWidth = 0.5842; // Distance between right and left wheels
      public static final double kWheelBase = 0.5842; // Distance between front and back wheels
      public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics( // Kinematics
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

      public static final double kMaxSpeedMetersPerSecond = 5.0; // Maxima Velocidad en Metros por Segundo
      public static final double kMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI; // Maxima Velocidad Angular en Radianes por Segundo

      public static final double kMaxAccelerationUnitsPerSecond = 3; // Maxima Aceleracion
      public static final double kMaxAngularAccelerationUnitsPerSecond = Math.PI / 4; // Maxima Aceleracion Angular

      public static final double kTeleopMaxSpeedMetersPerSecond = kMaxSpeedMetersPerSecond / 4; // Maxima Velocidad en Metros por Segundo
      public static final double kTeleopMaxAngularSpeedRadiansPerSecond = kMaxAngularSpeedRadiansPerSecond / 4; // Maxima Velocidad Angular en Radianes por Segundo

      public static final double kTeleopMaxAccelerationUnitsPerSecond = 3; // Maxima Aceleracion
      public static final double kTeleopMaxAngularAccelerationUnitsPerSecond = 3; // Maxima Aceleracion Angular

      public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kTeleopMaxAngularSpeedRadiansPerSecond, kTeleopMaxAngularAccelerationUnitsPerSecond);
      }

    public final static class Motors { // Motor IDs
      public static final Object[] kFrontLeftVars = { 
        0.0, // Offset
        false, // Inverted
        2, // Absolute Encoder ID
        6, // Drive Motor ID
        7, // Turning Motor ID
        true, // Drive Motor Inverted
        true, // Turning Motor Inverted
        "Front Left" // Name
      };
      public static final Object[] kBackLeftVars = {
        0.0, // Offset
        false, // Inverted
        4, // Absolute Encoder ID
        10, // Drive Motor ID
        11, // Turning Motor ID
        true, // Drive Motor Inverted
        true, // Turning Motor Inverted
        "Back Left" // Name
      };
      public static final Object[] kFrontRightVars = {
        0.0, // Offset
        false, // Inverted
        3, //[] Absolute Encoder ID
        8, // Drive Motor ID
        9, // Turning Motor ID
        false, // Drive Motor Inverted
        true, // Turning Motor Inverted
        "Front Right" // Name
      };
      public static final Object[] kBackRightVars = {
        0.0, // Offset
        false, // Inverted
        5, // Absolute Encoder ID
        12, // Drive Motor ID
        13, // Turning Motor ID
        false, // Drive Motor Inverted
        true, // Turning Motor Inverted
        "Back Right" // Name
      };
    }
  }

  public static final class Operator { // Operator Controllers and Data
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    public static final double kDeadzone = 0.1;
  }

}
