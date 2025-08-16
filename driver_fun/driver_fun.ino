#include <AccelStepper.h>
#include <HardwareSerial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// --- Configuración de motores ---
#define MOTOR_INTERFACE_TYPE 1
AccelStepper motor1(MOTOR_INTERFACE_TYPE, 14, 12);  // STEP: 14, DIR: 12 (ajusta pines)
AccelStepper motor2(MOTOR_INTERFACE_TYPE, 27, 26);
AccelStepper motor3(MOTOR_INTERFACE_TYPE, 33, 25);

// --- Variables globales (compartidas entre cores) ---
volatile float theta1, theta2, theta3, target_x, target_y;
volatile bool newDataAvailable = false;

// --- Task Handles ---
TaskHandle_t SerialTaskHandle;
TaskHandle_t MotorTaskHandle;

// --- Configuración ---
void setup() {
  Serial.begin(921600);  // Máxima velocidad serial

  // Configurar motores
  motor1.setMaxSpeed(2000);  // Pasos/segundo (micropasos)
  motor1.setAcceleration(1000);
  motor2.setMaxSpeed(2000);
  motor2.setAcceleration(1000);
  motor3.setMaxSpeed(2000);
  motor3.setAcceleration(1000);

  // Crear tareas en FreeRTOS
  xTaskCreatePinnedToCore(
    SerialTask,     // Función
    "SerialTask",   // Nombre
    10000,          // Stack size
    NULL,           // Parámetros
    1,              // Prioridad
    &SerialTaskHandle, 
    0               // Core 0
  );

  xTaskCreatePinnedToCore(
    MotorTask, 
    "MotorTask", 
    10000, 
    NULL, 
    2,              // Prioridad más alta
    &MotorTaskHandle, 
    1               // Core 1
  );
}

// --- Tarea para recibir datos serial ---
void SerialTask(void *pvParameters) {
  while (1) {
    if (Serial.available() > 0) {
      String data = Serial.readStringUntil('\n');
      parseData(data);
      newDataAvailable = true;
      Serial.write("ACK\n");  // Confirmación a la PC
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

// --- Tarea para mover motores ---
void MotorTask(void *pvParameters) {
  while (1) {
    if (newDataAvailable) {
      moveMotors(theta1, theta2, theta3);
      while (motor1.isRunning() || motor2.isRunning() || motor3.isRunning()) {
        motor1.run();
        motor2.run();
        motor3.run();
      }
      newDataAvailable = false;
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

// --- Funciones auxiliares (parseData y moveMotors igual que antes) ---