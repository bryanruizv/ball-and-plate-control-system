# Sistema de Control Bola y Placa (Ball and Plate System)

**Proyecto destacado:** Seleccionado para la **Feria de Proyectos de Fin de Ciclo (2025-2)** por su desempeño técnico y precisión en el control.

## 📄 Descripción
Este proyecto consiste en el diseño e implementación de un sistema de control de lazo cerrado capaz de estabilizar una esfera en el centro (o cualquier coordenada específica) de una superficie plana móvil. El sistema utiliza una placa resistiva para obtener la retroalimentación de la posición y servomotores para corregir la inclinación de la placa.

## ⚙️ Especificaciones Técnicas
* **Controlador:** Implementación de un algoritmo **PID** (Proporcional-Integral-Derivativo) para la estabilidad dinámica.
* **Sensores:** Uso de una placa resistiva de alta sensibilidad para la detección de posición en tiempo real.
* **Actuadores:** Servomotores de precisión integrados mediante señales de control PWM.
* **Plataforma de Desarrollo:** Programación de hardware en **Arduino IDE**.
* **Análisis de Datos:** Procesamiento de señales y monitoreo de rendimiento mediante **MATLAB**.

## 📊 Resultados y Logros
* **Tiempo de Asentamiento:** El sistema logra estabilizar la esfera en un tiempo promedio menor a **5 segundos**.
* **Reconocimiento Académico:** El proyecto fue seleccionado entre los mejores del curso de **Control Automático** para representar a la facultad en la feria de fin de ciclo.
* **Robustez:** Capacidad de respuesta ante perturbaciones externas manuales sobre la esfera.

## 🛠️ Fundamento Matemático
El control del sistema se basa en la aplicación de la ley de control PID para minimizar el error de posición:

$$u(t) = K_p e(t) + K_i \int_{0}^{t} e(\tau) d\tau + K_d \frac{de(t)}{dt}$$

## 📂 Documentación y Código
Puedes encontrar los archivos fuente de Arduino, los scripts de MATLAB y la documentación técnica detallada en este repositorio.
![Demostración Bola y Placa]
<video src="proyecto.mp4" controls="controls" style="max-width: 100%;">
</video>

