# LechuBOT

Robot móvil autónomo seguidor de línea y evasor de obstáculos, desarrollado con la plataforma Arduino.

## Descripción del Proyecto

LechuBOT es un robot móvil diseñado para navegar de forma autónoma siguiendo una línea negra en una superficie blanca. El sistema utiliza un controlador Proporcional-Derivativo (PD) para mantener su trayectoria y un sensor ultrasónico para detectar y evadir obstáculos de manera proactiva. El proyecto se enfoca en los principios fundamentales de la robótica móvil, incluyendo la percepción sensorial, los algoritmos de control y la actuación de motores.

## Componentes de Hardware

La construcción del robot se basa en una combinación de componentes estándar de robótica.

- **Microcontrolador:** 1x Arduino Mega 2560
- **Drivers de Motor:** 2x Driver L9110 (para controlar los 4 motores)
- **Sensores de Línea:** 3x Sensores Infrarrojos (IR) FC-51
- **Sensor de Distancia:** 1x Sensor Ultrasónico HC-SR04
- **Actuadores:** 4x Motoreductores DC con ruedas
- **Estructura:** 1x Chasis acrílico de doble nivel
- **Fuente de Alimentación:** Portapilas con 6 baterías AA (incluye 2 pilas adicionales para mayor amperaje)
- **Otros:** Sensor Shield, cables, tornillería.

## Software y Lógica del Robot

El firmware del robot está programado en C++ para Arduino y se organiza en varios archivos para mayor claridad y modularidad:

- `LineFollowerRobot.ino`: Archivo principal con las funciones `setup()` y `loop()`. Contiene la lógica central para alternar entre el seguimiento de línea y la evasión de obstáculos.
- `RobotLogic.cpp` / `RobotLogic.h`: Implementa la lógica del controlador PD para el seguimiento de línea y la maniobra de evasión de obstáculos.
- `MotorControl.cpp` / `MotorControl.h`: Contiene las funciones para inicializar y controlar los motores del robot.
- `config.h`: Define los pines de conexión y las constantes de control clave como `Kp`, `Kd`, y la compensación de motores (`LEFT_MOTOR_TRIM`).

### Lógica de Control de Línea (PD)

El robot sigue la línea utilizando un controlador PD que calcula un `error` basado en las lecturas de los tres sensores IR.

- **Error:** Se calcula un valor de `error` que indica la posición relativa del robot con respecto a la línea. El valor es 0 cuando el robot está centrado y aumenta negativamente (giro a la izquierda) o positivamente (giro a la derecha) a medida que se desvía.
- **Velocidad Adaptativa:** La velocidad base del robot se ajusta dinámicamente:
    - `140` para tramos rectos.
    - `110` para curvas suaves.
    - `90` para giros cerrados.
- **Point Turn:** En desviaciones extremas, el robot realiza un giro sobre su propio eje para volver rápidamente a la línea.

### Evasión de Obstáculos

Al detectar un obstáculo a menos de 20 cm, el robot ejecuta una maniobra de evasión programada en varios pasos para rodearlo y luego buscar activamente la línea para reanudar su trayectoria.

## Desafíos y Soluciones

Durante el desarrollo, se enfrentaron y superaron varios desafíos técnicos:

- **Fallo de Sensores IR:** Se detectaron sensores defectuosos que no distinguían entre blanco y negro. La solución fue reemplazarlos.
- **Desbalance de Motores:** La discrepancia de velocidad entre los motores izquierdos y derechos se corrigió con un factor de compensación (`LEFT_MOTOR_TRIM = 1.05`) en el software para balancear las velocidades.
- **Amperaje Insuficiente:** El robot experimentó un rendimiento deficiente debido a un amperaje insuficiente. La solución fue añadir dos baterías adicionales a la fuente de alimentación para asegurar el suministro de energía necesario.
- **Tuning del PD:** Se requirieron múltiples iteraciones de pruebas para ajustar las constantes `Kp` y `Kd` y lograr un equilibrio entre la reactividad del robot y la estabilidad en las curvas.
- **Maniobra de Evasión:** La lógica de evasión inicial era propensa a colisiones en ciertos escenarios. Se afinaron los tiempos y las velocidades de cada paso para mejorar la fiabilidad de la maniobra.
