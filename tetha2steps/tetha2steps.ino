#include <AccelStepper.h>

// Configuración de pines para los drivers DRV8825
#define MOTOR1_STEP_PIN 12
#define MOTOR1_DIR_PIN 14
#define MOTOR2_STEP_PIN 27
#define MOTOR2_DIR_PIN 26
#define MOTOR3_STEP_PIN 25
#define MOTOR3_DIR_PIN 33

// Configuración del motor (NEMA17 típico con DRV8825)
#define STEPS_PER_REV 200    // Pasos por revolución (1.8° por paso)
#define MICROSTEPS 16        // Microsteps configurados en el DRV8825
#define MAX_SPEED 1000.0    // Velocidad máxima en pasos/segundo
#define MAX_ACCEL 500.0      // Aceleración máxima en pasos/segundo^2

// Crear objetos de motor
AccelStepper motor1(AccelStepper::DRIVER, MOTOR1_STEP_PIN, MOTOR1_DIR_PIN);
AccelStepper motor2(AccelStepper::DRIVER, MOTOR2_STEP_PIN, MOTOR2_DIR_PIN);
AccelStepper motor3(AccelStepper::DRIVER, MOTOR3_STEP_PIN, MOTOR3_DIR_PIN);

// Variables para almacenar los ángulos objetivo
float targetAngle1 = 0, targetAngle2 = 0, targetAngle3 = 0;
bool newDataAvailable = false;

// Variables para el parsing serial
const byte numChars = 32;
char receivedChars[numChars];
boolean newData = false;

void setup() {
  Serial.begin(115200);
  
  // Configurar motores
  motor1.setMaxSpeed(MAX_SPEED);
  motor1.setAcceleration(MAX_ACCEL);
  
  motor2.setMaxSpeed(MAX_SPEED);
  motor2.setAcceleration(MAX_ACCEL);
  
  motor3.setMaxSpeed(MAX_SPEED);
  motor3.setAcceleration(MAX_ACCEL);
  
  // Habilitar los drivers (asumiendo que ENABLE está conectado a GND para habilitar)
  // Si usas pin de ENABLE, configúralo como OUTPUT y ponlo en LOW
  
  Serial.println("Robot Delta inicializado. Esperando comandos en formato: theta1,theta2,theta3");
}

void loop() {
  // Verificar si hay nuevos datos seriales
  recvWithEndMarker();
  
  // Si hay nuevos datos, procesarlos
  if (newData) {
    parseData();
    newData = false;
    newDataAvailable = true;
  }
  
  // Si hay nuevos ángulos, convertir a pasos y mover los motores
  if (newDataAvailable) {
    moveMotorsToTargetAngles();
    newDataAvailable = false;
  }
  
  // Ejecutar los motores
  motor1.run();
  motor2.run();
  motor3.run();
}

// Función para recibir datos seriales con terminador de nueva línea
void recvWithEndMarker() {
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;
  
  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();
    
    if (rc != endMarker) {
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= numChars) {
        ndx = numChars - 1;
      }
    }
    else {
      receivedChars[ndx] = '\0'; // terminar el string
      ndx = 0;
      newData = true;
    }
  }
}

// Función para parsear los ángulos recibidos
void parseData() {
  char * strtokIndx; // índice para strtok()
  
  strtokIndx = strtok(receivedChars, ",");
  targetAngle1 = atof(strtokIndx);
  
  strtokIndx = strtok(NULL, ",");
  targetAngle2 = atof(strtokIndx);
  
  strtokIndx = strtok(NULL, ",");
  targetAngle3 = atof(strtokIndx);
  
  Serial.print("Angulos recibidos - Theta1: ");
  Serial.print(targetAngle1);
  Serial.print("°, Theta2: ");
  Serial.print(targetAngle2);
  Serial.print("°, Theta3: ");
  Serial.print(targetAngle3);
  Serial.println("°");
}

// Función para convertir ángulos a pasos y mover los motores
void moveMotorsToTargetAngles() {
  // Convertir ángulos a pasos (considerando microsteps)
  long steps1 = angleToSteps(targetAngle1);
  long steps2 = angleToSteps(targetAngle2);
  long steps3 = angleToSteps(targetAngle3);
  
  // Mover motores a las nuevas posiciones
  motor1.moveTo(steps1);
  motor2.moveTo(steps2);
  motor3.moveTo(steps3);
  
  Serial.print("Movimiento iniciado - Pasos: ");
  Serial.print(steps1);
  Serial.print(", ");
  Serial.print(steps2);
  Serial.print(", ");
  Serial.println(steps3);
}

// Función para convertir ángulos a pasos (considerando microsteps)
long angleToSteps(float angle) {
  // Calcular pasos: (ángulo/360) * pasos_por_rev * microsteps
  return (angle / 360.0) * STEPS_PER_REV * MICROSTEPS;
}