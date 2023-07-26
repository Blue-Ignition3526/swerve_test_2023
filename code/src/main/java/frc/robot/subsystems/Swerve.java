// Codigo para crear el subsistema de Swerve con modulos, kinematica, y odometria
/////////////////////////////////////////////////////////////////////////////////
// Code to create the Swerve subsystem with modules, kinematics, and odometry

package frc.robot.subsystems; // Nombre del paquete donde se encuentra el archivo // Package name where the file is located

// Imports
import com.kauailabs.navx.frc.AHRS; // Libreria para la NAVX // Library for the NAVX
import org.team3526.lib.swerve.ModulePIDParameters;
import org.team3526.lib.swerve.SwerveModule; // Libreria para los modulos de Swerve // Library for Swerve modules
import frc.robot.Constants.Swerve.Motors; // Clase con los puertos de los motores y sus caracteristicas // Class with the motor ports and their characteristics
import frc.robot.Constants.Swerve.Physical; // Clase con las medidas fisicas del robot // Class with the physical measurements of the robot
import edu.wpi.first.math.geometry.Pose2d; // Libreria para la posicion // Library for the position
import edu.wpi.first.math.geometry.Rotation2d; // Libreria para la rotacion // Library for the rotation
import edu.wpi.first.math.kinematics.SwerveDriveKinematics; // Libreria para la kinematica de Swerve // Library for Swerve kinematics
import edu.wpi.first.math.kinematics.SwerveDriveOdometry; // Libreria para la odometria de Swerve // Library for Swerve odometry
import edu.wpi.first.math.kinematics.SwerveModulePosition; // Libreria para la posicion de los modulos // Library for the position of the modules
import edu.wpi.first.math.kinematics.SwerveModuleState; // Libreria para el estado de los modulos // Library for the state of the modules
import edu.wpi.first.wpilibj.SPI; // Libreria para el puerto SPI // Library for the SPI port
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; // Libreria para el SmartDashboard // Library for the SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase; // Libreria para el Subsistema // Library for the Subsystem

public class Swerve extends SubsystemBase {

  // Se declaran los modulos de Swerve // Swerve modules are declared
  private final SwerveModule m_frontLeft = new SwerveModule(Motors.kFrontLeftVars);
  private final SwerveModule m_frontRight = new SwerveModule(Motors.kFrontRightVars); 
  private final SwerveModule m_backLeft = new SwerveModule(Motors.kBackLeftVars);
  private final SwerveModule m_backRight = new SwerveModule(Motors.kBackRightVars);

  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP); // Se declara la NAVX // The NAVX is declared
  private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(Physical.kDriveKinematics, getRotation2d(), getSwervePositions()); // Se declara la odometria // Odometry is declared

  public Swerve() { 
    new Thread(() -> { // Se crea un hilo para resetear la NAVX // A thread is created to reset the NAVX
      try {
        Thread.sleep(1000); // Se espera un segundo // Wait a second
        resetGyro();
      } catch (Exception e) {
      }
    }).start();
  }

  public SwerveModulePosition[] getSwervePositions () { // Se obtienen las posiciones de los modulos // The positions of the modules are obtained
    return new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_backLeft.getPosition(),
      m_backRight.getPosition()
    };
  }

  public void resetGyro() { // Se resetea la NAVX // The NAVX is reset
    m_gyro.reset();
  }

  public double getAngle() { // Se obtiene el angulo de la NAVX // The angle of the NAVX is obtained
    return Math.IEEEremainder(m_gyro.getAngle(), 360);
  }

  public Rotation2d getRotation2d() { // Se obtiene la rotacion de la NAVX // The rotation of the NAVX is obtained
    return Rotation2d.fromDegrees(getAngle());
  }

  public void stopModules() { // Se detienen los modulos // The modules are stopped
    m_frontLeft.stop();
    m_frontRight.stop();
    m_backLeft.stop();
    m_backRight.stop();
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) { // Se establecen los estados de los modulos // The states of the modules are established
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Physical.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_backLeft.setDesiredState(desiredStates[2]);
    m_backRight.setDesiredState(desiredStates[3]);
  }

  public Pose2d getPose() { // Se obtiene la posicion // The position is obtained
    return odometer.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) { // Se resetea la odometria // Odometry is reset
    odometer.resetPosition(getRotation2d(), getSwervePositions(), pose);
  }

  @Override
  public void periodic() { // Se actualiza la odometria y se muestra en el SmartDashboard // Odometry is updated and displayed on the SmartDashboard
    ModulePIDParameters.m_PID_P = SmartDashboard.getNumber("PID_P", ModulePIDParameters.m_PID_P);
    ModulePIDParameters.m_PID_I = SmartDashboard.getNumber("PID_I", ModulePIDParameters.m_PID_I);
    ModulePIDParameters.m_PID_D = SmartDashboard.getNumber("PID_D", ModulePIDParameters.m_PID_D);

    odometer.update(getRotation2d(), getSwervePositions());
    SmartDashboard.putNumber("Robot Angle", getAngle());
    SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    SmartDashboard.putString("Robot Rotation", getRotation2d().toString());
  }
}
