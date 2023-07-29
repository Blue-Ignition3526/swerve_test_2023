// Comando para el subsistema de Swerve
////////////////////////////////////////
// Command for the Swerve subsystem

package frc.robot.commands.Swerve; 

// Imports
import java.util.function.Supplier; // Libreria para Supplier // Library for Supplier
import edu.wpi.first.math.filter.SlewRateLimiter; // Libreria para SlewRateLimiter // Library for SlewRateLimiter
import edu.wpi.first.math.kinematics.ChassisSpeeds; // Libreria para ChassisSpeeds // Library for ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveModuleState; // Libreria para SwerveModuleState // Library for SwerveModuleState
import edu.wpi.first.wpilibj2.command.CommandBase; // Libreria para CommandBase // Library for CommandBase
import frc.robot.Constants.Operator; // Libreria para Constantes de Operator // Library for Operator Constants
import frc.robot.Constants.Swerve.Physical; // Libreria para Constantes de Swerve de Fisica // Library for Swerve's Physics Constants' 
import frc.robot.subsystems.Swerve; // Importar el subsistema de Swerve // Import Swerve subsystem
public class Drive extends CommandBase {

  private final Swerve m_swerve; // Declarar el subsistema de Swerve // Declare Swerve subsystem
  private final Supplier<Double> xSpeed, ySpeed, rotSpeed; // Declarar los Suppliers // Declare Suppliers
  private final SlewRateLimiter xLimiter, yLimiter, rotLimiter; // Declarar los SlewRateLimiters // Declare SlewRateLimiters (Limitadores de aceleracion // Acceleration limiters)

  public Drive(Swerve m_swerve, Supplier<Double> xSpeed, Supplier<Double> ySpeed, Supplier<Double> rotSpeed) {
    this.m_swerve = m_swerve;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.rotSpeed = rotSpeed;
    this.xLimiter = new SlewRateLimiter(Physical.kTeleopMaxAccelerationUnitsPerSecond);
    this.yLimiter = new SlewRateLimiter(Physical.kTeleopMaxAccelerationUnitsPerSecond);
    this.rotLimiter = new SlewRateLimiter(Physical.kTeleopMaxAngularAccelerationUnitsPerSecond);
    addRequirements(m_swerve);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double xSpeed = this.xSpeed.get(); // Obtener los valores de los Suppliers // Get the values from the Suppliers
    double ySpeed = this.ySpeed.get();
    double rotSpeed = this.rotSpeed.get();

    xSpeed = Math.abs(xSpeed) > Operator.kDeadzone ? xSpeed : 0.0; // Aplicar la zona muerta // Apply deadzone
    ySpeed = Math.abs(ySpeed) > Operator.kDeadzone ? ySpeed : 0.0;
    rotSpeed = Math.abs(rotSpeed) > Operator.kDeadzone ? rotSpeed : 0.0;

    xSpeed = xLimiter.calculate(xSpeed) * Physical.kTeleopMaxSpeedMetersPerSecond; // Limitar la aceleracion // Limit acceleration
    ySpeed = yLimiter.calculate(ySpeed) * Physical.kTeleopMaxSpeedMetersPerSecond;
    rotSpeed = rotLimiter.calculate(rotSpeed) * Physical.kTeleopMaxAngularSpeedRadiansPerSecond;

    ChassisSpeeds m_chassisSpeeds;
    m_chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, m_swerve.getRotation2d()); // Aplicar la velocidad del chasis // Put chassis speed
    SwerveModuleState[] m_moduleStates = Physical.kDriveKinematics.toSwerveModuleStates(m_chassisSpeeds); // Calcular las velocidades de los modulos // Calculate modules' speeds
    m_swerve.setModuleStates(m_moduleStates); // Aplicar la velocidad de los modulos // Put modules speed
  }

  @Override
  public void end(boolean interrupted) {
    m_swerve.stopModules(); // Detener los modulos // Stop modules
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
