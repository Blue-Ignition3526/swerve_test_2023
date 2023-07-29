# Explicación del Codigo de Swerve Drive

## ¿Qué es un swerve drive?

Un swerve drive es un tipo de chassis *omnidireccional*. Nos permite movernos en todas las direcciones con facilidad. A diferencia de un chassis de *tanque* ,que solo puede moverse hacia adelante y hacia atrás, y girar en su propio eje, un swerve drive puede moverse en cualquier dirección y girar en su propio eje. Esto se logra mediante el uso de modulos que contienen dos motores, uno que gira la rueda y otro que la mueve hacia adelante y hacia atrás. Cada modulo tiene un encoder que nos permite saber la posición de la rueda.

![image](https://www.swervedrivespecialties.com/cdn/shop/products/MK4Image3_grande.jpg?v=1623269755) [^1]

En un chassis hay un total de 4 modulos, (uno en cada esquina).

[^1]: Modulo de swerve drive de Swerve Drive Specialties. [Link](https://www.swervedrivespecialties.com/products/mk4-swerve-module)

---

## Logica de Programación

Programar un chassis *Swerve* no es tan facil como programar uno *Diferencial*. En el diferencial basta con pasar valores a los dos lados del robot para lograr que se mueva. En el chassis de *Swerve* hay que calcular los angulos de las llantas, las velocidades de rotación para lograr el angulo deseado, asi como un controlador PID para asegurarse que el angulo deseado sea el angulo real. También hay que calcular la velocidad de las 4 llantas para asegurar estabilidad asi como un movimento deseado. Para esto, trabajamos con ***Cinemática y Odometria***.[^2]

[^2]: La cinemática (Kinematics) es el estudio del movimiento de los objetos. La odometria es el uso de valores para calcular la posicion del robot en la cancha.

---

## Programación

El primer paso para programar un chassis de *Swerve* es crear los 4 modulos. Para eso, en el codigo lo que hacemos es crear un archivo que sean un constructor para los modulos. Hay que recordar los componentes que conforman un modulo:
-   Motor de rotación
-   Motor de movimiento
-   Encoder de angulo

El constructor se encuentra en [src\main\java\org\team3526\lib\swerve\SwerveModule.java](src\main\java\org\team3526\lib\swerve\SwerveModule.java)

Lo  primero que hacemos es importar las librerias necesarias para el modulo. Para esto previamente se necesitan instalar las librerias de REV Robotics y CTRE Phoenix. Asi como tener el archivo de [*Constants*](src\main\java\frc\robot\Constants.java) que contiene los valores para calcular velocidades y rotaciones.

```java
// Inicio del archivo SwerveModule.java
package org.team3526.lib.swerve;

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
```

```java
// Inicio del archivo Constants.java

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class Constants {

  public final static class Swerve {

    public static final class Module {
      public static final double kWheelDiameterMeters = 0.1016;
      public static final double kDriveMotorGearRatio = 1.0 / 6.12;
      public static final double kTurningMotorGearRatio = 1.0 / 12.8;

      public static final double kDriveEncoder_RotationToMeter = kDriveMotorGearRatio * kWheelDiameterMeters * Math.PI; 
      public static final double kDriveEncoder_RPMToMeterPerSecond = kDriveEncoder_RotationToMeter / 60.0;

      public static final double kTurningEncoder_RotationToRadian = kTurningMotorGearRatio * 2.0 * Math.PI; 
      public static final double kTurningEncoder_RPMToRadianPerSecond = kTurningEncoder_RotationToRadian / 60.0;
    }

  }

}
```

El siguiente paso es declarar los dos motores, en este caso *SparkMax* de REV, asi como los valores de los encoders de los motores NEO. Creamos un controlador PID para la rotación del modulo. Declaramos el encoder de angulo, que en este caso es un *CANCoder* de CTRE. Declaramos un booleano para saber si el modulo esta invertido o no. El offset de angulo. Y por ultimo declaramos el nombre del modulo para que en la SmartDashboard se puedan identificar los valores.

También se agrega los valores del PID que se encuentran en [src\main\java\org\team3526\lib\swerve\ModulePIDParameters.java](src\main\java\org\team3526\lib\swerve\ModulePIDParameters.java). Esto para que se puedan actualizar los valores desde la SmartDashboard.

```java
package org.team3526.lib.swerve;

public class ModulePIDParameters {
    public static double m_PID_P = 0.5;
    public static double m_PID_I = 0.0;
    public static double m_PID_D = 0.0;
}
```

```java
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

  private double m_PID_P = ModulePIDParameters.m_PID_P;
  private double m_PID_I = ModulePIDParameters.m_PID_I;
  private double m_PID_D = ModulePIDParameters.m_PID_D;

}
```

Para crear un nuevo modulo de *Swerve* se necesita un arreglo de objetos que contenga los siguientes valores:

```java
public static final Object[] kFrontLeftVars = { 
    0, // Offset
    false, // Inverted
    2, // Absolute Encoder ID
    6, // Drive Motor ID
    7, // Turning Motor ID
    true, // Drive Motor Inverted
    true, // Turning Motor Inverted
    "Front Left" // Name
    };
```

Con el siguiente codigo se crea un modulo de *Swerve* con los valores del arreglo de objetos.

```java
public class SwerveModule extends SubsystemBase {
  public SwerveModule(Object[] Arr) { // Constructor
    this.m_turningEncoderOffsetRad = (double) Arr[0];
    this.m_turningAbsoluteEncoderInverted = (boolean) Arr[1];
    m_turningAbsoluteEncoder = new CANCoder((int) Arr[2]);
    // Es necesario tener el encoder con su valor absoluto para que el modulo siempre sepa su rotacion en 360 grados, sin importar si el robot es deshabilitado o apagado.

    m_driveMotor = new CANSparkMax((int) Arr[3], MotorType.kBrushless);
    m_turningMotor = new CANSparkMax((int) Arr[4], MotorType.kBrushless);

    m_driveMotor.setInverted((boolean) Arr[5]); 
    m_turningMotor.setInverted((boolean) Arr[6]); 

    m_driveEncoder = m_driveMotor.getEncoder();
    m_turningEncoder = m_turningMotor.getEncoder(); 

    // Configuracion de los encoders (Equivalencias de ticks de encoders a medidas
    m_driveEncoder.setPositionConversionFactor(Swerve.Module.kDriveEncoder_RotationToMeter); // Conversion de rotacion a metros
    m_driveEncoder.setVelocityConversionFactor(Swerve.Module.kDriveEncoder_RPMToMeterPerSecond); // Conversion de RPM a metros por segundo
    m_turningEncoder.setPositionConversionFactor(Swerve.Module.kTurningEncoder_RotationToRadian); // Conversion de rotacion a radianes
    m_turningEncoder.setVelocityConversionFactor(Swerve.Module.kTurningEncoder_RPMToRadianPerSecond); // Conversion de RPM a radianes por segundo 

    m_turningPIDController = new PIDController(m_PID_P, m_PID_I, m_PID_D);
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI); // Habilitar la entrada continua

    m_name = (String) Arr[7]; 

    resetEncoders();
  }
}
```
---

### Los modulos contienen las siguientes funciones:

***`getPosition()`*** - Retorna la posición del modulo en la clase *SwerveModulePosition*, la cual consiste de la posición (Metros avanzados) y la rotación (Radianes girados)

```java	
public SwerveModulePosition getPosition() { 
  return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));
}
```

***`getDrivePosition()`*** - Retorna la posición del motor de avance en metros

```java	
public double getDrivePosition() { 
  return m_driveEncoder.getPosition();
}
```

***`getTurningPosition()`*** - Retorna la posición del motor de giro en radianes

```java	
public double getTurningPosition() {
  return m_turningEncoder.getPosition();
}
```

***`getDriveVelocity()`*** - Retorna la velocidad del motor de avance en metros por segundo

```java	
public double getDriveVelocity() {
  return m_driveEncoder.getVelocity();
}
```

***`getTurningVelocity()`*** - Retorna la velocidad del motor de giro en radianes por segundo

```java	
public double getTurningVelocity() {
  return m_turningEncoder.getVelocity();
}
```

***`getAbsoluteEncoderRad()`*** - Retorna la posición absoluta del encoder de giro en radianes. Es necesario obtener el valor absoluto del encoder para que el modulo siempre sepa su rotación en 360 grados, sin importar si el robot es deshabilitado o apagado.

```java	
public double getAbsoluteEncoderRad() {
  double angle = m_turningAbsoluteEncoder.getAbsolutePosition();
  angle *= 2.0 * Math.PI;
  angle -= m_turningEncoderOffsetRad;
  return angle * (m_turningAbsoluteEncoderInverted ? -1.0 : 1.0);
}
```

***`resetEncoders()`*** - Restablece los encoders de avance a 0 y de giro a su posición absoluta

```java	
public void resetEncoders() {
  m_driveEncoder.setPosition(0);
  m_turningEncoder.setPosition(getAbsoluteEncoderRad());
}
```

***`getState()`*** - Retorna el estado del modulo en la clase *SwerveModuleState*, la cual consiste de la velocidad de avance (Metros por segundo) y la posición de giro (Radianes)

```java	
public SwerveModuleState getState() {
  return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
}
```

***`stop()`*** - Detiene los motores de avance y de giro

```java	
public void stop() {
  m_driveMotor.set(0);
  m_turningMotor.set(0);
}
```

***`setDesiredState()`*** - Mueve el modulo a un estado deseado en la clase *SwerveModuleState*, la cual consiste de la velocidad de avance (Metros por segundo) y la posición de giro (Radianes). Primero se revisa si la velocidad de avance es menor a 0.001, si es así, se detienen los motores. Si no, se optimiza el estado deseado para girar la menor cantidad de grados y se mueven los motores de avance y de giro a la velocidad y posición deseadas respectivamente.

```java	
public void setDesiredState(SwerveModuleState state) {
  if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      stop();
      return;
  }
  state = SwerveModuleState.optimize(state, getState().angle);
  m_driveMotor.set(state.speedMetersPerSecond / Swerve.Physical.kMaxSpeedMetersPerSecond);
  m_turningMotor.set(m_turningPIDController.calculate(getTurningPosition(), state.angle.getRadians()));
}
```

Para terminar el Modulo, en ***`periodic()`*** se invluye lo siguiente:
- Se obtienen los valores del controlador PID de la clase *ModulePIDParameters*.
- Los valores a mostrar en la SmartDashboard.

```java
@Override
public void periodic () {
  m_PID_P = ModulePIDParameters.m_PID_P;
  m_PID_I = ModulePIDParameters.m_PID_I;
  m_PID_D = ModulePIDParameters.m_PID_D;

  SmartDashboard.putString(m_name+" Module State", getState().toString());
  SmartDashboard.putString(m_name+" Module Position", getPosition().toString());
  SmartDashboard.putNumber(m_name+" Module Speed", getDriveVelocity());
  SmartDashboard.putNumber(m_name+" Module Turning Speed", getTurningVelocity());
  SmartDashboard.putNumber(m_name+" Drive Encoder", getDrivePosition());
  SmartDashboard.putNumber(m_name+" Turning Encoder", getTurningPosition());
  SmartDashboard.putNumber(m_name+" Turning Absolute Encoder", getAbsoluteEncoderRad());
}
```

Eso seria todo el codigo del modulo, lo siguiente es juntar los 4 modulos en el subsistema *[Swerve](src\main\java\frc\robot\subsystems\Swerve.java)*.
