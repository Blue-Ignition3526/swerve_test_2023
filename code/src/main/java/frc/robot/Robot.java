// Robot.java es el archivo que corre todo el codigo del robot, la base de command-based es un timed-robot,
// que es una clase que corre el codigo en un loop cada 20ms, y tiene funciones para cada modo del robot.
// Si se puede modificar el archivo, pero no es recomendable, ya que es la base del codigo del robot.
//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Robot.java is the file that runs all the robot code, the base of command-based is a timed-robot,
// which is a class that runs the code in a loop every 20ms, and has functions for each mode of the robot.
// If you can modify the file, but it is not recommended, since it is the base of the robot code.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  // Codigo de robot que se corre al iniciar el robot
  // Robot code that runs when the robot is started
  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }

  // Codigo de robot que se corre periodicamente, cada 20ms. 
  // Robot code that runs periodically, every 20ms.
  @Override
  public void robotPeriodic() {
    // Command scheduler corre los comandos declarados en RobotContainer
    // Command scheduler runs the commands declared in RobotContainer
    CommandScheduler.getInstance().run();
    // Iniciar DataLog para guardar datos en un archivo
    // Start DataLog to save data in a file
    DataLogManager.start();
    // Capturar datos de la Driver Station
    // Capture data from the Driver Station
    DriverStation.startDataLog(DataLogManager.getLog());
  }

  // Codigo que corre una vez cuando el robot se deshabilita
  // Code that runs once when the robot is disabled
  @Override
  public void disabledInit() {}

  // Codigo que corre periodicamente cuando el robot se deshabilita
  // Code that runs periodically when the robot is disabled
  @Override
  public void disabledPeriodic() {}

  // Codigo que corre cuando se inicia el modo autonomo
  // Code that runs when autonomous mode starts
  @Override
  public void autonomousInit() {
    // Se obtiene el comando autonomo de RobotContainer
    // Autonomous command is fetched from RobotContainer
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // Si hay un comando autonomo, se agrega a la lista de comandos a correr
    // If there is an autonomous command, it is added to the list of commands to run
    if (m_autonomousCommand != null) {m_autonomousCommand.schedule();}
  }

  // Codigo que corre periodicamente cuando el robot esta en modo autonomo
  // Code that runs periodically when the robot is in autonomous mode
  @Override
  public void autonomousPeriodic() {}

  // Codigo que corre cuando se inicia el modo teleoperado
  // Code that runs when teleoperated mode starts
  @Override
  public void teleopInit() {
    // Se cancela el modo autonomo una vez inciado el modo teleoperado
    // Se puede comentar esta linea si se quiere que el robot siga corriendo el modo autonomo

    // Autonomous mode is cancelled once teleoperated mode starts
    // This line can be commented if you want the robot to keep running the autonomous mode
    if (m_autonomousCommand != null) {m_autonomousCommand.cancel();}
  }

  // Codigo que corre periodicamente cuando el robot esta en modo teleoperado
  // Code that runs periodically when the robot is in teleoperated mode
  @Override
  public void teleopPeriodic() {}

  // Codigo que corre cuando se inicia el modo test
  // Code that runs when test mode starts
  @Override
  public void testInit() {
    // Se cancelan todos los comandos corriendo al iniciar el modo test
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  // Codigo que corre periodicamente cuando el robot esta en modo test
  // Code that runs periodically when the robot is in test mode
  @Override
  public void testPeriodic() {}

  // Codigo que corre cuando se inicia el modo simulacion
  // Code that runs when simulation mode starts
  @Override
  public void simulationInit() {}

  // Codigo que corre periodicamente cuando el robot esta en modo simulacion
  // Code that runs periodically when the robot is in simulation mode
  @Override
  public void simulationPeriodic() {}
}
