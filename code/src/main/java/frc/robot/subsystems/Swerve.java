// Codigo para crear el subsistema de Swerve con modulos, kinematica, y odometria
/////////////////////////////////////////////////////////////////////////////////
// Code to create the Swerve subsystem with modules, kinematics, and odometry

package frc.robot.subsystems; 

// Imports
import com.kauailabs.navx.frc.AHRS;
import org.team3526.lib.swerve.SwerveModule; 
import frc.robot.Constants.Swerve.Motors; 
import frc.robot.Constants.Swerve.Physical; 
import edu.wpi.first.math.geometry.Pose2d; 
import edu.wpi.first.math.geometry.Rotation2d; 
import edu.wpi.first.math.kinematics.SwerveDriveKinematics; 
import edu.wpi.first.math.kinematics.SwerveDriveOdometry; 
import edu.wpi.first.math.kinematics.SwerveModulePosition; 
import edu.wpi.first.math.kinematics.SwerveModuleState; 
import edu.wpi.first.wpilibj.I2C; 
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; 
import edu.wpi.first.wpilibj2.command.SubsystemBase; 

public class Swerve extends SubsystemBase {

  private final SwerveModule m_frontLeft = new SwerveModule(Motors.kFrontLeftVars);
  private final SwerveModule m_frontRight = new SwerveModule(Motors.kFrontRightVars); 
  private final SwerveModule m_backLeft = new SwerveModule(Motors.kBackLeftVars);
  private final SwerveModule m_backRight = new SwerveModule(Motors.kBackRightVars);

  private final AHRS m_gyro = new AHRS(I2C.Port.kMXP); 
  private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(Physical.kDriveKinematics, getRotation2d(), getSwervePositions()); 

  public Swerve() { 
    new Thread(() -> {
      try {
        Thread.sleep(1000); 
        resetGyro();
      } catch (Exception e) {
      }
    }).start();
  }

  /**
   * Constructs an array of the swerve module positions
   * @return the swerve module positions
   */
  public SwerveModulePosition[] getSwervePositions () { 
    return new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_backLeft.getPosition(),
      m_backRight.getPosition()
    };
  }

  /**
  * Resets the gyro to 0 degrees  
  */
  public void resetGyro() { 
    m_gyro.reset();
  }

  /**
   * Gets the gyro angle in degrees
   * @return the gyro angle in degrees
   */
  public double getAngle() { 
    return m_gyro.getYaw()%360;
  }

  /**
   * Converts the gyro angle into a Rotation2d object
   * @return the angle in Rotation2d
   */
  public Rotation2d getRotation2d() { 
    return Rotation2d.fromDegrees(getAngle());
  }

  /**
   * Stops all the swerve modules
   */
  public void stopModules() { 
    m_frontLeft.stop();
    m_frontRight.stop();
    m_backLeft.stop();
    m_backRight.stop();
  }

  /**
   * Sets the desired states of the swerve modules
   * @param desiredStates the desired states of the swerve modules
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) { 
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Physical.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_backLeft.setDesiredState(desiredStates[2]);
    m_backRight.setDesiredState(desiredStates[3]);
  }

  /**
   * Gets the current pose of the robot
   * @return the current pose of the robot
   */
  public Pose2d getPose() { 
    return odometer.getPoseMeters();
  }

  /**
   * Resets the odometry to a specified pose
   * @param pose the pose to reset the odometry to
   */
  public void resetOdometry(Pose2d pose) {
    odometer.resetPosition(getRotation2d(), getSwervePositions(), pose);
  }

  @Override
  public void periodic() { 
    odometer.update(getRotation2d(), getSwervePositions());
    SmartDashboard.putNumber("Robot Angle", getAngle());
    SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    SmartDashboard.putString("Robot Rotation", getRotation2d().toString());
  }
}
