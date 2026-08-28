# TFG: Firmware del Nodo de Adquisición de Datos (STM32)

Este repositorio almacena el código fuente en C programado en el microcontrolador (STM32G431RB) para el Trabajo de Fin de Grado centrado en telemetría de sistemas VIPV (energía solar integrada en vehículos).

Este bloque de código representa la inteligencia física del prototipo a bordo del vehículo y se encarga de tres tareas fundamentales:
1. **Adquisición de sensórica:** Lectura e interpretación continua de los sensores de irradiancia, temperatura, potencia y aceleración inercial. 
2. **Control lógico:** Ejecución del algoritmo adaptativo de seguimiento del punto de máxima potencia (MPPT), optimizado para evitar caídas de rendimiento frente a las sombras dinámicas de la conducción.
3. **Comunicaciones:** Empaquetado de la telemetría e interacción directa con la red CAN del vehículo para extraer datos de tracción mediante el protocolo OBD-II.

---
Omar Ftouh Labrouzi - Trabajo de Fin de Grado (Ingeniería Electrónica Industrial y Automática, Universidad Politécnica de Madrid).
