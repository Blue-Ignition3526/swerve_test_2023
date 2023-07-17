// El archivo principal del robot, aqui se declaran los subsistemas, comandos y controles.
///////////////////////////////////////////////////////////////////////////////////////////
// The main file of the robot, subsystems, commands and controls are declared here.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  // Aqui se declaran los subsistemas, comandos y controladores
  // Subsystems, commands and controllers are declared here
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem(); ///////////

  // Declaracion de un control de xbox (Se puede cambiar a PS4 o Joystick)
  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  // Declaracion de los Triggers
  // Triggers are declared here
  // private final Trigger m_oc_aButton = m_operatorController.a(); ///////////

  // Declaracion de commandos por defecto y mappeos del control
  // Default commands and control bindings are declared here
  public RobotContainer() {


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
    return null;
  }
}
