// Codigo para declarar un solo modulo de Swerve
/////////////////////////////////////////////////
// Code to declare a single Swerve module

package org.team3526.lib.swerve; 
 
import frc.robot.Constants;
// Imports
import frc.robot.Constants.Swerve;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType; 
import edu.wpi.first.math.controller.PIDController; 
import edu.wpi.first.math.geometry.Rotation2d; 
import edu.wpi.first.math.kinematics.SwerveModulePosition; 
import edu.wpi.first.math.kinematics.SwerveModuleState; 
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase; 

public class SwerveModule extends SubsystemBase {

  private final CANSparkMax m_driveMotor; 
  private final CANSparkMax m_turningMotor; 

  private final RelativeEncoder m_turningEncoder; 
  private final RelativeEncoder m_driveEncoder; 

  private final PIDController m_turningPIDController;

  private final CANCoder m_turningAbsoluteEncoder; 
  private final boolean m_turningAbsoluteEncoderInverted; 
  private final double m_turningEncoderOffsetRad; 

  private final String m_name; 

  public SwerveModule(Object[] Arr) { 
    m_turningEncoderOffsetRad = (double) Arr[0]; 
    m_turningAbsoluteEncoderInverted = (boolean) Arr[1]; 
    m_turningAbsoluteEncoder = new CANCoder((int) Arr[2]); 

    m_driveMotor = new CANSparkMax((int) Arr[3], MotorType.kBrushless); 
    m_turningMotor = new CANSparkMax((int) Arr[4], MotorType.kBrushless);

    m_driveMotor.setInverted((boolean) Arr[5]); 
    m_turningMotor.setInverted((boolean) Arr[6]); 

    m_driveEncoder = m_driveMotor.getEncoder(); 
    m_turningEncoder = m_turningMotor.getEncoder();

    m_driveEncoder.setPositionConversionFactor(Swerve.Module.kDriveEncoder_RotationToMeter); 
    m_driveEncoder.setVelocityConversionFactor(Swerve.Module.kDriveEncoder_RPMToMeterPerSecond);
    m_turningEncoder.setPositionConversionFactor(Swerve.Module.kTurningEncoder_RotationToRadian); 
    m_turningEncoder.setVelocityConversionFactor(Swerve.Module.kTurningEncoder_RPMToRadianPerSecond); 

    m_turningPIDController = new PIDController(0.5, 0, 0);
    m_turningPIDController.enableContinuousInput(0.0, 2.0*Math.PI); 
    m_name = (String) Arr[7];

    resetEncoders();
  }

  /**
   * Get the current position of the swerve module
   * @return The position of the swerve module
   */
  public SwerveModulePosition getPosition() { 
    return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));
  }

  /**
   * Get the distance in meters that the module has driven
   * @return The distance in meters
   */
  public double getDrivePosition() {
    return m_driveEncoder.getPosition();
  }

  /**
   * Get the angle in radians that the module has turned
   * @return The angle in radians
   */
  public double getTurningPosition() { 
    return m_turningEncoder.getPosition();
  }

  /**
   * Get the velocity of the drive motor in meters per second
   * @return The velocity in m/s
   */
  public double getDriveVelocity() {
      return m_driveEncoder.getVelocity();
  }

  /**
   * Get the velocity of the turning motor in radians per second
   * @return The velocity in rad/s
   */
  public double getTurningVelocity() { 
      return m_turningEncoder.getVelocity();
  }

  /**
   * Gets the position of the swerve module's absolute encoder in radians
   * @return The absolute angle of the wheel in radians
   */
  public double getAbsoluteEncoderRad() {
      double angle = m_turningAbsoluteEncoder.getAbsolutePosition();
      angle = Math.toRadians(angle);
      angle -= m_turningEncoderOffsetRad;
      return angle * (m_turningAbsoluteEncoderInverted ? -1.0 : 1.0);
  }

  /** 
  * Reset the drive encoder to a position of 0 and
  * Reset the turning encoder to the absolute encoder's position 
  */
  public void resetEncoders() {
      m_driveEncoder.setPosition(0);
      m_turningEncoder.setPositionConversionFactor(Constants.Swerve.Module.kTurningEncoder_RotationToRadian);
      m_turningEncoder.setPosition(getAbsoluteEncoderRad());
  }

  /**
   * Get the state of the swerve module
   * @return A SwerveModuleState representing the speed and angle of the module
   */
  public SwerveModuleState getState() {
      return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
  }

  /**
   * Set the desired state of the swerve module
   * @param state The desired state of the swerve module
   */
  public void setDesiredState(SwerveModuleState state) { 
      if (Math.abs(state.speedMetersPerSecond) < 0.001) {
          stop();
          return;
      }
      state = SwerveModuleState.optimize(state, getState().angle); 
      m_driveMotor.set(state.speedMetersPerSecond / Swerve.Physical.kMaxSpeedMetersPerSecond);
      m_turningMotor.set(m_turningPIDController.calculate(getTurningPosition(), state.angle.getRadians()));
  }

  /**
   * Stop the swerve module from moving
   */
  public void stop() { 
      m_driveMotor.set(0);
      m_turningMotor.set(0);
  }
  
  @Override
  public void periodic () { // Periodo de ejecucion // Execution period
    SmartDashboard.putString(m_name+" Module State", getState().toString());
    SmartDashboard.putString(m_name+" Module Position", getPosition().toString());
    SmartDashboard.putNumber(m_name+" Module Speed", getDriveVelocity());
    SmartDashboard.putNumber(m_name+" Module Turning Speed", getTurningVelocity());
    SmartDashboard.putNumber(m_name+" Drive Encoder", getDrivePosition());
    SmartDashboard.putNumber(m_name+" Turning Encoder", getTurningPosition());
    SmartDashboard.putNumber(m_name+" Turning Absolute Encoder Direct", m_turningAbsoluteEncoder.getAbsolutePosition());
    SmartDashboard.putNumber(m_name+" Turning Absolute Encoder", getAbsoluteEncoderRad());
    SmartDashboard.putData(m_name+" PID CONTROLLER",m_turningPIDController);
  }
}
