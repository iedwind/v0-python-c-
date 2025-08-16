 #include <AccelStepper.h>

// // --- CONFIGURACIÓN DE LA PRUEBA ---
// // Cambia estos valores para experimentar
 float MAX_SPEED_TEST = 120000.0;
 float ACCELERATION_TEST = 112000.0;
// float MOVE_ANGLE = 45.0; // El ángulo al que se moverá (en grados)

// // --- PINES Y PARÁMETROS DEL MOTOR (ajusta si son diferentes) ---

// //EJE Y (PCB)
 #define DIR1_PIN  4
 #define STEP1_PIN 2
// //EJE X (PCB)
// #define DIR2_PIN  32
// #define STEP2_PIN 12
// //EJE A (PCB)
// #define DIR3_PIN  17
// #define STEP3_PIN 16
 const int STEPS_PER_REV = 200;
 const int GEARBOX_RATIO = 5;
 const int MICROSTEPPING = 32; // Asegúrate que este valor sea correcto (16, 32, etc.)
 const float TOTAL_STEPS_PER_REV = STEPS_PER_REV * GEARBOX_RATIO * MICROSTEPPING;
 const float STEPS_PER_DEGREE = TOTAL_STEPS_PER_REV / 360.0;

// // Crear el objeto para el motor
// AccelStepper motor1(AccelStepper::DRIVER, STEP1_PIN, DIR1_PIN);
// long target_steps = MOVE_ANGLE * STEPS_PER_DEGREE;

// void setup() {
//     Serial.begin(115200);
//     Serial.println("Iniciando prueba de tuning para Motor 1...");

//     // Configurar el motor con los valores de prueba
//     motor1.setMaxSpeed(MAX_SPEED_TEST);
//     motor1.setAcceleration(ACCELERATION_TEST);

//     // Invertir dirección si es necesario (basado en nuestras pruebas anteriores)
//     //motor1.setPinsInverted(true, false, false);

//     // Mover a la posición inicial
//     motor1.moveTo(target_steps);
// }

// void loop() {
//     // Si el motor ha llegado a su destino...
//     if (motor1.distanceToGo() == 0) {
//         // ...invertir la dirección del movimiento
//         motor1.moveTo(-motor1.currentPosition());
//         Serial.print("Llegó a la posición. Moviendo a: ");
//         Serial.println(motor1.targetPosition() / STEPS_PER_DEGREE);
//     }

//     // Esta función debe llamarse constantemente
//     motor1.run();
// }




const int REPETITIONS = 50; // Haremos 50 ciclos de ida y vuelta
const float START_ANGLE = 0.0;
const float END_ANGLE = 45.0;

// --- PINES Y PARÁMETROS (Sin cambios) ---

AccelStepper motor1(1, STEP1_PIN, DIR1_PIN);
void setup() {
    Serial.begin(115200);
    Serial.println("Iniciando prueba de repetibilidad (Corregida)...");

    motor1.setMaxSpeed(MAX_SPEED_TEST);
    motor1.setAcceleration(ACCELERATION_TEST);
    motor1.setPinsInverted(true, false, false);

    motor1.setCurrentPosition(0); // Asegurarse de que el motor piense que está en la posición 0
    
    Serial.print("Moviendo 50 veces entre ");
    Serial.print(START_ANGLE);
    Serial.print(" y ");
    Serial.print(END_ANGLE);
    Serial.println(" grados...");

    // Iniciar el primer movimiento
    motor1.moveTo(END_ANGLE * STEPS_PER_DEGREE);
}

void loop() {
    static int cycles_completed = 0;

    // Solo ejecutar si no hemos completado todas las repeticiones
    if (cycles_completed < REPETITIONS) {
        // Mover el motor constantemente
        motor1.run();

        // Si el motor ha llegado a su destino...
        if (motor1.distanceToGo() == 0) {
            // Chequear si acabamos de llegar al ángulo final (END_ANGLE)
            if (motor1.currentPosition() != 0) {
                // Si es así, el siguiente movimiento es de regreso al inicio
                motor1.moveTo(START_ANGLE * STEPS_PER_DEGREE);
            } 
            // Chequear si acabamos de llegar al inicio (START_ANGLE)
            else {
                // Si es así, hemos completado un ciclo completo de "ida y vuelta"
                cycles_completed++;
                Serial.print("Ciclo completado: ");
                Serial.println(cycles_completed);

                // Si aún no hemos terminado todas las repeticiones, iniciar el siguiente ciclo
                if (cycles_completed < REPETITIONS) {
                    motor1.moveTo(END_ANGLE * STEPS_PER_DEGREE);
                }
            }
        }
    } else {
        // Al final de la prueba, imprimir mensaje y no hacer nada más
        if (cycles_completed == REPETITIONS) {
           Serial.println("\nPrueba finalizada. Verifica la posición final del brazo.");
           cycles_completed++; // Incrementar para no volver a entrar aquí y sobrecargar el serial
        }
    }
}