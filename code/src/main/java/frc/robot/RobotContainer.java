// El archivo principal del robot, aqui se declaran los subsistemas, comandos y controles.
///////////////////////////////////////////////////////////////////////////////////////////
// The main file of the robot, subsystems, commands and controls are declared here.

package frc.robot;

import org.team3526.lib.swerve.TrajectoryGenerator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Operator;
import frc.robot.commands.Drive;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
  // Aqui se declaran los subsistemas, comandos y controladores
  // Subsystems, commands and controllers are declared here
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem(); ///////////

  private final Swerve m_swerve = new Swerve();

  // Declaracion de un control de xbox (Se puede cambiar a PS4 o Joystick)
  private final CommandXboxController m_driverController = new CommandXboxController(Operator.kDriverControllerPort);

  // Declaracion de los Triggers
  // Triggers are declared here
  // private final Trigger m_oc_aButton = m_operatorController.a(); ///////////

  // Declaracion de commandos por defecto y mappeos del control
  // Default commands and control bindings are declared here
  public RobotContainer() {
    // Al subsistema de Swerve se le asigna el comando de Drive por default y se pasan los parametros
    m_swerve.setDefaultCommand(new Drive(m_swerve, () -> m_driverController.getLeftX(), () -> m_driverController.getLeftY(), () -> m_driverController.getRightX()));

    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    // Aqui se mapean los comandos a los botones del control
    // Commands are binded to controller buttons here
    // m_oc_aButton.whileTrue(new ExampleCommand(m_exampleSubsystem)); ///////////
    
    
  }

  // Aqui se regresa el comando autonomo a ejecutar
  // Here the autonomous command to run is returned
  public Command getAutonomousCommand() {
    SwerveControllerCommand autonPath = TrajectoryGenerator.newTrajectory("paths/Basic.wpilib.json", m_swerve);
    return autonPath;
  }
}
