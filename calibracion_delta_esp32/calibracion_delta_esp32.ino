#include <AccelStepper.h>

// --- CONFIGURACIÓN (Usa los mismos valores de tu sketch principal) ---
#define DIR1_PIN  13
#define STEP1_PIN 12
#define DIR2_PIN  14
#define STEP2_PIN 27
#define DIR3_PIN  26
#define STEP3_PIN 25

const float MAX_SPEED = 120000.0;
const float ACCELERATION = 112000.0;
const int STEPS_PER_REV = 200;
const int GEARBOX_RATIO = 5;
const int MICROSTEPPING = 32;
const float STEPS_PER_DEGREE = (STEPS_PER_REV * GEARBOX_RATIO * MICROSTEPPING) / 360.0;

AccelStepper motor1(AccelStepper::DRIVER, STEP1_PIN, DIR1_PIN);
AccelStepper motor2(AccelStepper::DRIVER, STEP2_PIN, DIR2_PIN);
AccelStepper motor3(AccelStepper::DRIVER, STEP3_PIN, DIR3_PIN);

char rxBuffer[64];

void setup() {
    Serial.begin(115200);
    
    motor1.setMaxSpeed(MAX_SPEED);
    motor1.setAcceleration(ACCELERATION);
    motor2.setMaxSpeed(MAX_SPEED);
    motor2.setAcceleration(ACCELERATION);
    motor3.setMaxSpeed(MAX_SPEED);
    motor3.setAcceleration(ACCELERATION);

    // No olvides invertir los pines si fue necesario en tus pruebas
    motor1.setPinsInverted(true, false, false);
    motor2.setPinsInverted(true, false, false);
    motor3.setPinsInverted(true, false, false);
}

void loop() {
    // Escuchar siempre por nuevos comandos seriales
    if (Serial.available() > 0) {
        static byte index = 0;
        char receivedChar = Serial.read();

        if (receivedChar == '<') { 
            index = 0; 
        } else if (receivedChar == '>') {
            rxBuffer[index] = '\0'; // Terminar el string
            
            float t1, t2, t3;
            // Parsear el string para obtener los 3 ángulos
            char* token = strtok(rxBuffer, ","); if(token) t1 = atof(token);
            token = strtok(NULL, ","); if(token) t2 = atof(token);
            token = strtok(NULL, ","); if(token) t3 = atof(token);

            // Enviar los motores a los nuevos ángulos objetivo
            motor1.moveTo(t1 * STEPS_PER_DEGREE);
            motor2.moveTo(t2 * STEPS_PER_DEGREE);
            motor3.moveTo(t3 * STEPS_PER_DEGREE);
        } else if (index < sizeof(rxBuffer) - 1) {
            rxBuffer[index++] = receivedChar;
        }
    }

    // Mover los motores constantemente hacia su objetivo
    motor1.run();
    motor2.run();
    motor3.run();
}4