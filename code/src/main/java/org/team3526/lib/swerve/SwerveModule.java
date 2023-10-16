// Codigo para declarar un solo modulo de Swerve
/////////////////////////////////////////////////
// Code to declare a single Swerve module

package org.team3526.lib.swerve; // Nombre del paquete donde se encuentra el modulo // Package name where the module is located

// Imports
import frc.robot.Constants.Swerve; // Importar las constantes de Swerve // Import Swerve constants
import com.ctre.phoenix.sensors.CANCoder; // Importar el CANCoder // Import CANCoder
import com.revrobotics.CANSparkMax; // Importar el CANSparkMax // Import CANSparkMax
import com.revrobotics.RelativeEncoder; // Importar el RelativeEncoder // Import RelativeEncoder
import com.revrobotics.CANSparkMaxLowLevel.MotorType; // Importar los tipos de motor // Import motor types
import edu.wpi.first.math.controller.PIDController; // Importar el PIDController // Import PIDController
import edu.wpi.first.math.geometry.Rotation2d; // Importar el Rotation2d // Import Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition; // Importar el SwerveModulePosition // Import SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState; // Importar el SwerveModuleState // Import SwerveModuleState
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase; // Importar el SubsystemBase // Import SubsystemBase

public class SwerveModule extends SubsystemBase {

  private final CANSparkMax m_driveMotor; // Motor de manejo // Drive motor
  private final CANSparkMax m_turningMotor; // Motor de giro // Turning motor

  private final RelativeEncoder m_turningEncoder; // Encoder de giro // Turning encoder
  private final RelativeEncoder m_driveEncoder; // Encoder de manejo // Drive encoder

  private final PIDController m_turningPIDController; // Controlador PID de giro // Turning PID controller

  private final CANCoder m_turningAbsoluteEncoder; // Encoder absoluto de giro // Turning absolute encoder
  private final boolean m_turningAbsoluteEncoderInverted; // Invertir el encoder absoluto de giro // Invert turning absolute encoder
  private final double m_turningEncoderOffsetRad; // Offset del encoder de giro // Turning encoder offset

  private final String m_name; // Nombre del modulo // Module name

  private double m_PID_P = ModulePIDParameters.m_PID_P; // Constante P del PID de giro // Turning PID P constant
  private double m_PID_I = ModulePIDParameters.m_PID_I; // Constante I del PID de giro // Turning PID I constant
  private double m_PID_D = ModulePIDParameters.m_PID_D; // Constante D del PID de giro // Turning PID D constant

  private double m_startingAbsoluteAngle;

  public SwerveModule(Object[] Arr) { // Constructor del modulo de Swerve // Swerve module constructor
    m_turningEncoderOffsetRad = (double) Arr[0]; // Offset del encoder de giro // Turning encoder offset (valor 1//value 1)
    m_turningAbsoluteEncoderInverted = (boolean) Arr[1]; // Invertir el encoder absoluto de giro // Invert turning absolute encoder (valor 2//value 2)
    m_turningAbsoluteEncoder = new CANCoder((int) Arr[2]); // Encoder absoluto de giro // Turning absolute encoder (valor 3//value 3)

    m_driveMotor = new CANSparkMax((int) Arr[3], MotorType.kBrushless); // Motor de manejo // Drive motor (valor 4//value 4)
    m_turningMotor = new CANSparkMax((int) Arr[4], MotorType.kBrushless); // Motor de giro // Turning motor (valor 5//value 5)

    m_driveMotor.setInverted((boolean) Arr[5]); // Invertir el motor de manejo // Invert drive motor (valor 6//value 6)
    m_turningMotor.setInverted((boolean) Arr[6]); // Invertir el motor de giro // Invert turning motor (valor 7//value 7)

    m_driveEncoder = m_driveMotor.getEncoder(); // Encoder de manejo // Drive encoder
    m_turningEncoder = m_turningMotor.getEncoder(); // Encoder de giro // Turning encoder

    // Configuracion de los encoders // Encoder configuration (Equivalencias de ticks de encoders a medidas // Encoder ticks to measurements equivalences)
    m_driveEncoder.setPositionConversionFactor(Swerve.Module.kDriveEncoder_RotationToMeter); // Conversion de rotacion a metros // Rotation to meters conversion
    m_driveEncoder.setVelocityConversionFactor(Swerve.Module.kDriveEncoder_RPMToMeterPerSecond); // Conversion de RPM a metros por segundo // RPM to meters per second conversion
    m_turningEncoder.setPositionConversionFactor(Swerve.Module.kTurningEncoder_RotationToRadian); // Conversion de rotacion a radianes // Rotation to radians conversion
    m_turningEncoder.setVelocityConversionFactor(Swerve.Module.kTurningEncoder_RPMToRadianPerSecond); // Conversion de RPM a radianes por segundo // RPM to radians per second conversion

    m_turningPIDController = new PIDController(m_PID_P, m_PID_I, m_PID_D); // Configuracion del PID de giro // Turning PID configuration
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI); // Habilitar la entrada continua // Enable continuous input
    // Habilita que la llanta gire 360 grados // Enable 360 degrees wheel rotation
    m_name = (String) Arr[7]; // Nombre del modulo de Swerve // Swerve module name (valor 8//value 8)

    m_startingAbsoluteAngle = Math.toRadians(m_turningAbsoluteEncoder.getAbsolutePosition());

    SmartDashboard.putNumber(m_name + " Starting Angle", m_startingAbsoluteAngle);

    resetEncoders(); // Reiniciar los encoders // Reset encoders
  }

  public double getStartingAbsoluteAngle() {
    return m_startingAbsoluteAngle;
  }

  public SwerveModulePosition getPosition() { // Obtener la posicion del modulo de Swerve // Get Swerve module position (Metros avanzados y giro // Meters moved and rotation)
    return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));
  }

  public double getDrivePosition() { // Obtener la posicion del motor de manejo // Get drive motor position (Metros avanzados // Meters moved)
    return m_driveEncoder.getPosition();
  }

  public double getTurningPosition() { // Obtener la posicion del motor de giro // Get turning motor position (Radianes girados // Radians rotated)
    return m_turningEncoder.getPosition() % (2 * Math.PI);
  }

  public double getDriveVelocity() { // Obtener la velocidad del motor de manejo // Get drive motor velocity (Metros por segundo // Meters per second)
      return m_driveEncoder.getVelocity();
  }

  public double getTurningVelocity() { // Obtener la velocidad del motor de giro // Get turning motor velocity (Radianes por segundo // Radians per second)
      return m_turningEncoder.getVelocity();
  }

  public double getAbsoluteEncoderRad() { // Obtener la posicion del encoder absoluto de giro // Get turning absolute encoder position (Radianes girados // Radians rotated)
      double angle = m_turningAbsoluteEncoder.getAbsolutePosition();
      angle = Math.toRadians(angle);
      if (m_turningAbsoluteEncoderInverted) {
        return angle * -1;
      } else {
        return angle;
      }
  }

  public void resetEncoders() { // Reiniciar los encoders // Reset encoders
      m_driveEncoder.setPosition(0);
      m_turningEncoder.setPosition(0);
  }

  public SwerveModuleState getState() { // Obtener el estado del modulo de Swerve // Get Swerve module state (Velocidad de manejo y giro // Drive and turning velocity)
      return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
  }

  public void setDesiredState(SwerveModuleState state) { // Establecer el estado deseado del modulo de Swerve // Set Swerve module desired state 
      if (Math.abs(state.speedMetersPerSecond) < 0.001) {
          stop();
          return;
      }
      state = new SwerveModuleState(state.speedMetersPerSecond, state.angle.minus(new Rotation2d(m_startingAbsoluteAngle + m_turningEncoderOffsetRad)));
      state = SwerveModuleState.optimize(state, getState().angle); // Optimizar el estado deseado para girar la menor cantidad de grados // Optimize desired state to rotate the least amount of degrees
      m_driveMotor.set(state.speedMetersPerSecond / Swerve.Physical.kMaxSpeedMetersPerSecond);
      m_turningMotor.set(m_turningPIDController.calculate(getTurningPosition(), state.angle.getRadians()));
  }

  public void stop() { // Detener el modulo de Swerve // Stop Swerve module
      m_driveMotor.set(0);
      m_turningMotor.set(0);
  }
  
  @Override
  public void periodic () { // Periodo de ejecucion // Execution period
    m_PID_P = ModulePIDParameters.m_PID_P;
    m_PID_I = ModulePIDParameters.m_PID_I;
    m_PID_D = ModulePIDParameters.m_PID_D;

    SmartDashboard.putString(m_name+" Module State", getState().toString());
    SmartDashboard.putString(m_name+" Module Position", getPosition().toString());
    SmartDashboard.putNumber(m_name+" Module Speed", getDriveVelocity());
    SmartDashboard.putNumber(m_name+" Module Turning Speed", getTurningVelocity());
    SmartDashboard.putNumber(m_name+" Drive Encoder", getDrivePosition());
    SmartDashboard.putNumber(m_name+" Turning Encoder", getTurningPosition());
    SmartDashboard.putNumber(m_name+" Turning Absolute Encoder Direct", m_turningAbsoluteEncoder.getAbsolutePosition());
    SmartDashboard.putNumber(m_name+" Turning Absolute Encoder", getAbsoluteEncoderRad());
  }
}
