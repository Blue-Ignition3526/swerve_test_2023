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
