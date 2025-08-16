// #include <AccelStepper.h>
// #define motorInterfaceType 1 
// // --- 1. CONFIGURACIÓN DE PINES Y PARÁMETROS ---
// // Define los pines STEP y DIR para cada uno de los 3 motores
// //EJE Y (PCB)
// #define DIR1_PIN  4
// #define STEP1_PIN 2
// //EJE X (PCB)
// #define DIR2_PIN  32
// #define STEP2_PIN 12
// //EJE A (PCB)
// #define DIR3_PIN  17
// #define STEP3_PIN 16

// // Pin para controlar la bomba de succión (a través de un relé o MOSFET)
// #define PUMP_PIN 27

// // --- 2. PARÁMETROS DE LOS MOTORES (ajusta si son diferentes) ---
// const int STEPS_PER_REV = 200;    // Pasos por revolución del motor NEMA 17 (típicamente 200)
// const int GEARBOX_RATIO = 5;      // Relación de la caja reductora (5:1)
// const int MICROSTEPPING = 16;     // Micropasos configurados en el DRV8825 (ej: 1, 2, 4, 8, 16, 32)

// // Cálculo automático de pasos totales y pasos por grado
// const float TOTAL_STEPS_PER_REV = STEPS_PER_REV * GEARBOX_RATIO * MICROSTEPPING;
// const float STEPS_PER_DEGREE = TOTAL_STEPS_PER_REV / 360.0;

// // --- 3. CONFIGURACIÓN DE MOVIMIENTO (¡Tuning requerido!) ---
// // Experimenta con estos valores para obtener un movimiento rápido pero estable
// const float MAX_SPEED = 4000.0;     // Velocidad máxima en pasos/segundo
// const float ACCELERATION = 2000.0;  // Aceleración en pasos/segundo^2

// // --- 4. POSICIONES PRECALCULADAS (en ángulos) ---
// // Calcula estas posiciones usando tu script de Python para las coordenadas (x,y,z) deseadas
// // y anótalas aquí.
// const float HOME_THETAS[3] = {0.0, 0.0, 0.0};             // Posición de reposo segura
// const float Z_DROP_OFFSET_DEGREES = 5.0; // Ángulo extra para bajar un poco en los contenedores

// // Coordenadas para los contenedores (DEBES CALCULARLAS)
// // Ejemplo: Mueve el robot manualmente a la posición de soltar, anota los ángulos.
// const float LEFT_CONTAINER_THETAS[3] = {-35.0, 18.0, 18.0};  // Contenedor izquierdo (cuadrados)
// const float RIGHT_CONTAINER_THETAS[3] = {35.0, -18.0, -18.0}; // Contenedor derecho (círculos)

// // --- 5. OBJETOS Y VARIABLES GLOBALES ---
// AccelStepper motor1(motorInterfaceType, STEP1_PIN, DIR1_PIN);
// AccelStepper motor2(motorInterfaceType, STEP2_PIN, DIR2_PIN);
// AccelStepper motor3(motorInterfaceType, STEP3_PIN, DIR3_PIN);


// char rxBuffer[64];
// bool newData = false;

// float targetThetas[3];
// char destination;


// // --- 6. FUNCIONES DE CONTROL ---

// void moveToAngles(const float t1, const float t2, const float t3) {
//     // Convierte los ángulos a pasos absolutos
//     long targetSteps1 = t1 * STEPS_PER_DEGREE;
//     long targetSteps2 = t2 * STEPS_PER_DEGREE;
//     long targetSteps3 = t3 * STEPS_PER_DEGREE;

//     // Asigna el objetivo a cada motor
//     motor1.moveTo(targetSteps1);
//     motor2.moveTo(targetSteps2);
//     motor3.moveTo(targetSteps3);

//     // Bucle de bloqueo: espera a que todos los motores lleguen a su destino
//     // run() debe ser llamado constantemente para que los motores se muevan.
//     while (motor1.distanceToGo() != 0 || motor2.distanceToGo() != 0 || motor3.distanceToGo() != 0) {
//         motor1.run();
//         motor2.run();
//         motor3.run();
//     }
// }

// void executePickAndPlace() {
//     Serial.println("-> Iniciando secuencia Pick & Place...");

//     // 1. Moverse a la posición sobre el objeto para recogerlo
//     Serial.println("1. Moviendo para recoger objeto...");
//     moveToAngles(targetThetas[0], targetThetas[1], targetThetas[2]);
//     delay(200);

//     // 2. Activar la bomba de succión
//     Serial.println("2. Activando succión...");
//     digitalWrite(PUMP_PIN, HIGH);
//     delay(500); // Dar tiempo a que se genere el vacío

//     // 3. Elevar el objeto (regresar a HOME o a una altura segura)
//     Serial.println("3. Elevando objeto...");
//     moveToAngles(HOME_THETAS[0], HOME_THETAS[1], HOME_THETAS[2]);
//     delay(200);

//     // 4. Mover al contenedor correcto (izquierda o derecha)
//     const float* containerThetas = (destination == 'L') ? LEFT_CONTAINER_THETAS : RIGHT_CONTAINER_THETAS;
//     Serial.print("4. Moviendo a contenedor: "); Serial.println(destination);
//     moveToAngles(containerThetas[0], containerThetas[1], containerThetas[2]);
    
//     // 5. Bajar un poco para soltar el objeto sin chocar
//     moveToAngles(containerThetas[0], containerThetas[1] + Z_DROP_OFFSET_DEGREES, containerThetas[2] + Z_DROP_OFFSET_DEGREES);
//     delay(200);

//     // 6. Desactivar la bomba de succión
//     Serial.println("5. Soltando objeto...");
//     digitalWrite(PUMP_PIN, LOW);
//     delay(500); // Dar tiempo a que el objeto se desprenda

//     // 7. Subir a la altura del contenedor antes de moverse
//     moveToAngles(containerThetas[0], containerThetas[1], containerThetas[2]);
//     delay(200);

//     // 8. Regresar a la posición de inicio (HOME)
//     Serial.println("6. Regresando a HOME...");
//     moveToAngles(HOME_THETAS[0], HOME_THETAS[1], HOME_THETAS[2]);

//     // 9. Enviar confirmación a la PC de que la tarea ha finalizado
//     Serial.println("DONE");
//     Serial.println("-> Secuencia completada. Esperando nuevo comando.");
// }


// // --- 7. SETUP Y LOOP PRINCIPAL ---

// void setup() {
//     Serial.begin(115200);

//     // Configurar los motores con su velocidad y aceleración
//     motor1.setMaxSpeed(MAX_SPEED);
//     motor1.setAcceleration(ACCELERATION);
//     motor2.setMaxSpeed(MAX_SPEED);
//     motor2.setAcceleration(ACCELERATION);
//     motor3.setMaxSpeed(MAX_SPEED);
//     motor3.setAcceleration(ACCELERATION);
    
//     // Configurar el pin de la bomba como salida y apagarla
//     pinMode(PUMP_PIN, OUTPUT);
//     digitalWrite(PUMP_PIN, LOW);

//     // Homing simplificado: Asumimos que el robot empieza en la posición 0,0,0
//     // En un sistema real, se necesitarían finales de carrera (endstops) para un homing preciso.
//     motor1.setCurrentPosition(0);
//     motor2.setCurrentPosition(0);
//     motor3.setCurrentPosition(0);

//     Serial.println("Robot Delta inicializado. Esperando comandos...");
// }

// void loop() {
//     // Revisar si hay datos nuevos en el puerto serie
//     if (Serial.available() > 0) {
//         static byte index = 0;
//         char receivedChar = Serial.read();

//         if (receivedChar == '<') { // Caracter de inicio de comando
//             index = 0;
//         } else if (receivedChar == '>') { // Caracter de fin de comando
//             rxBuffer[index] = '\0'; // Terminar el string
            
//             // Parsear (dividir) el string: "t1,t2,t3,dest"
//             char* token = strtok(rxBuffer, ",");
//             if(token) targetThetas[0] = atof(token);
            
//             token = strtok(NULL, ",");
//             if(token) targetThetas[1] = atof(token);
            
//             token = strtok(NULL, ",");
//             if(token) targetThetas[2] = atof(token);
            
//             token = strtok(NULL, ",");
//             if(token) destination = token[0];
            
//             newData = true; // Activar la bandera de que hay un nuevo comando
//         } else {
//             if (index < sizeof(rxBuffer) - 1) {
//                 rxBuffer[index++] = receivedChar;
//             }
//         }
//     }

//     // Si hay un nuevo comando válido, ejecutar la secuencia de movimientos
//     if (newData) {
//         newData = false; // Bajar la bandera para no ejecutarlo de nuevo
//         executePickAndPlace();
//     }
    
//     // NOTA: No se llaman a los motor.run() aquí porque la función moveToAngles es bloqueante.
//     // Si se quisiera un control no bloqueante, los motor.run() deberían estar aquí en el loop
//     // y la función executePickAndPlace debería ser una máquina de estados más compleja.
//     // Para este prototipo, el enfoque bloqueante es más simple y efectivo.
// }


#include <AccelStepper.h>

// --- 1. CONFIGURACIÓN DE PINES Y PARÁMETROS (igual que antes) ---
#define motorInterfaceType 1 

//EJE Y (PCB)
#define DIR1_PIN  4
#define STEP1_PIN 2
//EJE X (PCB)
#define DIR2_PIN  32
#define STEP2_PIN 12
//EJE A (PCB)
#define DIR3_PIN  17
#define STEP3_PIN 16

#define PUMP_PIN 27 

const int STEPS_PER_REV = 200;
const int GEARBOX_RATIO = 5;
const int MICROSTEPPING = 32;
const float TOTAL_STEPS_PER_REV = STEPS_PER_REV * GEARBOX_RATIO * MICROSTEPPING;
const float STEPS_PER_DEGREE = TOTAL_STEPS_PER_REV / 360.0;

const float MAX_SPEED = 120000.0;
const float ACCELERATION = 112000.0;

//--- 2. POSICIONES PRECALCULADAS (igual que antes) ---
const float HOME_THETAS[3] = {0.0, 0.0, 0.0};
const float Z_DROP_OFFSET_DEGREES = 5.0;

const float LEFT_CONTAINER_THETAS[3] = {-70, -62, -30};
const float RIGHT_CONTAINER_THETAS[3] = {-30, -62, -70};

// --- 3. NUEVA SECCIÓN: MÁQUINA DE ESTADOS FINITA (FSM) ---
// Definimos todos los estados posibles de nuestro robot
enum EstadoRobot {
  IDLE,                // Esperando un nuevo comando desde Python
  MOVING_TO_PICK,      // Moviéndose a la posición de recogida
  ACTIVATING_PUMP,     // Esperando a que la bomba de succión genere vacío
  MOVING_TO_HOME_WITH_OBJ, // Elevando el objeto a una posición segura
  MOVING_TO_DROP,      // Moviéndose hacia el contenedor
  LOWERING_TO_DROP,    // Bajando un poco para soltar
  DEACTIVATING_PUMP,   // Esperando a que el objeto se suelte
  RAISING_AFTER_DROP,  // Subiendo de nuevo
  RETURNING_HOME       // Regresando a la posición de inicio
};

EstadoRobot estadoActual = IDLE; // El estado inicial del robot
unsigned long tiempoDeEspera = 0; // Para manejar esperas sin usar delay()
const unsigned int PUMP_WAIT_MS = 500; // 500ms de espera para la bomba

// --- 4. OBJETOS Y VARIABLES GLOBALES ---
AccelStepper motor1(motorInterfaceType, STEP1_PIN, DIR1_PIN);
AccelStepper motor2(motorInterfaceType, STEP2_PIN, DIR2_PIN);
AccelStepper motor3(motorInterfaceType, STEP3_PIN, DIR3_PIN);

char rxBuffer[64];
float pickThetas[3];
char destination;

// --- 5. SETUP (casi igual) ---
void setup() {
    Serial.begin(115200);

    motor1.setMaxSpeed(MAX_SPEED);
    motor1.setAcceleration(ACCELERATION);
    motor2.setMaxSpeed(MAX_SPEED);
    motor2.setAcceleration(ACCELERATION);
    motor3.setMaxSpeed(MAX_SPEED);
    motor3.setAcceleration(ACCELERATION);
    
    pinMode(PUMP_PIN, OUTPUT);
    digitalWrite(PUMP_PIN, LOW);

    motor1.setCurrentPosition(0);
    motor2.setCurrentPosition(0);
    motor3.setCurrentPosition(0);

    Serial.println("Robot Delta (FSM) inicializado. Esperando comandos...");
}

// --- 6. FUNCIONES DE AYUDA ---
void setTargetAngles(const float t1, const float t2, const float t3) {
    motor1.moveTo(t1 * STEPS_PER_DEGREE);
    motor2.moveTo(t2 * STEPS_PER_DEGREE);
    motor3.moveTo(t3 * STEPS_PER_DEGREE);
}

bool movementIsComplete() {
    return (motor1.distanceToGo() == 0 && motor2.distanceToGo() == 0 && motor3.distanceToGo() == 0);
}

// --- 7. LOOP PRINCIPAL CON MÁQUINA DE ESTADOS ---
void loop() {
    // Estas funciones DEBEN llamarse en cada ciclo del loop para que los motores se muevan.
    motor1.run();
    motor2.run();
    motor3.run();

    // El motor de la Máquina de Estados
    switch (estadoActual) {
        case IDLE:
            // En estado IDLE, solo escuchamos por un nuevo comando.
            if (Serial.available() > 0) {
                // ... (lógica de recepción serial, igual que antes) ...
                static byte index = 0;
                char receivedChar = Serial.read();
                if (receivedChar == '<') { index = 0; } 
                else if (receivedChar == '>') {
                    rxBuffer[index] = '\0';
                    char* token = strtok(rxBuffer, ","); if(token) pickThetas[0] = atof(token);
                    token = strtok(NULL, ","); if(token) pickThetas[1] = atof(token);
                    token = strtok(NULL, ","); if(token) pickThetas[2] = atof(token);
                    token = strtok(NULL, ","); if(token) destination = token[0];
                    
                    Serial.println("-> Comando recibido. Iniciando secuencia...");
                    setTargetAngles(pickThetas[0], pickThetas[1], pickThetas[2]);
                    estadoActual = MOVING_TO_PICK; // ¡Transición al siguiente estado!
                } else if (index < sizeof(rxBuffer) - 1) {
                    rxBuffer[index++] = receivedChar;
                }
            }
            break;

        case MOVING_TO_PICK:
            // Esperar a que el movimiento se complete.
            if (movementIsComplete()) {
                Serial.println("1. Posición de recogida alcanzada. Activando bomba...");
                digitalWrite(PUMP_PIN, HIGH);
                tiempoDeEspera = millis(); // Iniciar el temporizador para la bomba
                estadoActual = ACTIVATING_PUMP;
            }
            break;

        case ACTIVATING_PUMP:
            // Esperar un tiempo definido sin usar delay().
            if (millis() - tiempoDeEspera >= PUMP_WAIT_MS) {
                Serial.println("2. Objeto succionado. Elevando...");
                setTargetAngles(HOME_THETAS[0], HOME_THETAS[1], HOME_THETAS[2]);
                estadoActual = MOVING_TO_HOME_WITH_OBJ;
            }
            break;
        
        case MOVING_TO_HOME_WITH_OBJ:
            if (movementIsComplete()) {
                Serial.print("3. Moviendo a contenedor: "); Serial.println(destination);
                const float* containerThetas = (destination == 'L') ? LEFT_CONTAINER_THETAS : RIGHT_CONTAINER_THETAS;
                setTargetAngles(containerThetas[0], containerThetas[1], containerThetas[2]);
                estadoActual = MOVING_TO_DROP;
            }
            break;
            
        case MOVING_TO_DROP:
             if (movementIsComplete()) {
                Serial.println("4. Bajando para soltar...");
                const float* containerThetas = (destination == 'L') ? LEFT_CONTAINER_THETAS : RIGHT_CONTAINER_THETAS;
                setTargetAngles(containerThetas[0], containerThetas[1] + Z_DROP_OFFSET_DEGREES, containerThetas[2] + Z_DROP_OFFSET_DEGREES);
                estadoActual = LOWERING_TO_DROP;
            }
            break;

        case LOWERING_TO_DROP:
            if (movementIsComplete()) {
                Serial.println("5. Soltando objeto...");
                digitalWrite(PUMP_PIN, LOW);
                tiempoDeEspera = millis(); // Iniciar temporizador para que caiga
                estadoActual = DEACTIVATING_PUMP;
            }
            break;

        case DEACTIVATING_PUMP:
            if (millis() - tiempoDeEspera >= PUMP_WAIT_MS) {
                Serial.println("6. Elevando efector...");
                const float* containerThetas = (destination == 'L') ? LEFT_CONTAINER_THETAS : RIGHT_CONTAINER_THETAS;
                setTargetAngles(containerThetas[0], containerThetas[1], containerThetas[2]);
                estadoActual = RAISING_AFTER_DROP;
            }
            break;

        case RAISING_AFTER_DROP:
             if (movementIsComplete()) {
                Serial.println("7. Regresando a HOME...");
                setTargetAngles(HOME_THETAS[0], HOME_THETAS[1], HOME_THETAS[2]);
                estadoActual = RETURNING_HOME;
            }
            break;

        case RETURNING_HOME:
            if (movementIsComplete()) {
                Serial.println("-> Secuencia completada. Listo para nuevo comando.");
                Serial.println("DONE"); // Enviar confirmación a Python
                estadoActual = IDLE; // Volver al estado inicial
            }
            break;
    }
}



// const int STEPS_PER_REV = 200;
// const int GEARBOX_RATIO = 5;
// const int MICROSTEPPING = 32;
// const float TOTAL_STEPS_PER_REV = STEPS_PER_REV * GEARBOX_RATIO * MICROSTEPPING;
// const float STEPS_PER_DEGREE = TOTAL_STEPS_PER_REV / 360.0;
// const float MAX_SPEED = 6000.0; // Podemos probar a aumentar la velocidad
// const float ACCELERATION = 3000.0; // Y la aceleración

// const float HOME_THETAS[3] = {0.0, 0.0, 0.0};
// const float Z_DROP_OFFSET_DEGREES = 5.0;
// const float LEFT_CONTAINER_THETAS[3] = {-35.0, 18.0, 18.0};
// const float RIGHT_CONTAINER_THETAS[3] = {35.0, -18.0, -18.0};

// // --- MÁQUINA DE ESTADOS (FSM) ---
// // Se elimina el estado RETURNING_HOME del ciclo principal
// enum EstadoRobot {
//   IDLE,
//   MOVING_TO_PICK,
//   ACTIVATING_PUMP,
//   MOVING_TO_DROP, // Se va directo a soltar, sin pasar por HOME
//   LOWERING_TO_DROP,
//   DEACTIVATING_PUMP,
//   RAISING_AFTER_DROP
// };

// EstadoRobot estadoActual = IDLE;
// unsigned long tiempoDeEspera = 0;
// const unsigned int PUMP_WAIT_MS = 400; // Reducimos un poco el tiempo

// // --- OBJETOS Y VARIABLES GLOBALES (Sin cambios) ---
// AccelStepper motor1(motorInterfaceType, STEP1_PIN, DIR1_PIN);
// AccelStepper motor2(motorInterfaceType, STEP2_PIN, DIR2_PIN);
// AccelStepper motor3(motorInterfaceType, STEP3_PIN, DIR3_PIN);
// char rxBuffer[64];
// float pickThetas[3];
// char destination;

// // --- SETUP (Sin cambios) ---
// void setup() {
//     Serial.begin(115200);
//     motor1.setMaxSpeed(MAX_SPEED);
//     motor1.setAcceleration(ACCELERATION);
//     motor2.setMaxSpeed(MAX_SPEED);
//     motor2.setAcceleration(ACCELERATION);
//     motor3.setMaxSpeed(MAX_SPEED);
//     motor3.setAcceleration(ACCELERATION);
//     pinMode(PUMP_PIN, OUTPUT);
//     digitalWrite(PUMP_PIN, LOW);
//     motor1.setCurrentPosition(0);
//     motor2.setCurrentPosition(0);
//     motor3.setCurrentPosition(0);
//     Serial.println("Robot Delta (FSM optimizado) inicializado.");
// }

// // --- FUNCIONES DE AYUDA (Sin cambios) ---
// void setTargetAngles(const float t1, const float t2, const float t3) {
//     motor1.moveTo(t1 * STEPS_PER_DEGREE);
//     motor2.moveTo(t2 * STEPS_PER_DEGREE);
//     motor3.moveTo(t3 * STEPS_PER_DEGREE);
// }

// bool movementIsComplete() {
//     return (motor1.distanceToGo() == 0 && motor2.distanceToGo() == 0 && motor3.distanceToGo() == 0);
// }

// // --- LOOP PRINCIPAL OPTIMIZADO ---
// void loop() {
//     motor1.run();
//     motor2.run();
//     motor3.run();

//     switch (estadoActual) {
//         case IDLE:
//             if (Serial.available() > 0) {
//                 // Lógica de recepción serial (sin cambios)
//                 static byte index = 0;
//                 char receivedChar = Serial.read();
//                 if (receivedChar == '<') { index = 0; } 
//                 else if (receivedChar == '>') {
//                     rxBuffer[index] = '\0';
//                     char* token = strtok(rxBuffer, ","); if(token) pickThetas[0] = atof(token);
//                     token = strtok(NULL, ","); if(token) pickThetas[1] = atof(token);
//                     token = strtok(NULL, ","); if(token) pickThetas[2] = atof(token);
//                     token = strtok(NULL, ","); if(token) destination = token[0];
                    
//                     Serial.println("-> Comando recibido. Moviendo a recoger...");
//                     setTargetAngles(pickThetas[0], pickThetas[1], pickThetas[2]);
//                     estadoActual = MOVING_TO_PICK;
//                 } else if (index < sizeof(rxBuffer) - 1) {
//                     rxBuffer[index++] = receivedChar;
//                 }
//             }
//             break;

//         case MOVING_TO_PICK:
//             if (movementIsComplete()) {
//                 Serial.println("1. En posición. Activando bomba...");
//                 digitalWrite(PUMP_PIN, HIGH);
//                 tiempoDeEspera = millis();
//                 estadoActual = ACTIVATING_PUMP;
//             }
//             break;

//         case ACTIVATING_PUMP:
//             if (millis() - tiempoDeEspera >= PUMP_WAIT_MS) {
//                 Serial.print("2. Objeto recogido. Moviendo a contenedor: "); Serial.println(destination);
//                 // ### OPTIMIZACIÓN: IR DIRECTO AL CONTENEDOR ###
//                 const float* containerThetas = (destination == 'L') ? LEFT_CONTAINER_THETAS : RIGHT_CONTAINER_THETAS;
//                 setTargetAngles(containerThetas[0], containerThetas[1], containerThetas[2]);
//                 estadoActual = MOVING_TO_DROP;
//             }
//             break;
            
//         case MOVING_TO_DROP:
//              if (movementIsComplete()) {
//                 Serial.println("3. Bajando para soltar...");
//                 const float* containerThetas = (destination == 'L') ? LEFT_CONTAINER_THETAS : RIGHT_CONTAINER_THETAS;
//                 setTargetAngles(containerThetas[0], containerThetas[1] + Z_DROP_OFFSET_DEGREES, containerThetas[2] + Z_DROP_OFFSET_DEGREES);
//                 estadoActual = LOWERING_TO_DROP;
//             }
//             break;

//         case LOWERING_TO_DROP:
//             if (movementIsComplete()) {
//                 Serial.println("4. Soltando objeto...");
//                 digitalWrite(PUMP_PIN, LOW);
//                 tiempoDeEspera = millis();
//                 estadoActual = DEACTIVATING_PUMP;
//             }
//             break;

//         case DEACTIVATING_PUMP:
//             if (millis() - tiempoDeEspera >= PUMP_WAIT_MS) {
//                 Serial.println("5. Elevando efector...");
//                 const float* containerThetas = (destination == 'L') ? LEFT_CONTAINER_THETAS : RIGHT_CONTAINER_THETAS;
//                 setTargetAngles(containerThetas[0], containerThetas[1], containerThetas[2]);
//                 estadoActual = RAISING_AFTER_DROP;
//             }
//             break;

//         case RAISING_AFTER_DROP:
//              if (movementIsComplete()) {
//                 // ### OPTIMIZACIÓN CLAVE ###
//                 // El robot está en una posición segura sobre el contenedor.
//                 // Notificamos a la PC que estamos listos para el siguiente comando AHORA.
//                 Serial.println("6. Secuencia completada. ¡Listo para nuevo comando!");
//                 Serial.println("DONE"); 
//                 estadoActual = IDLE; // Volvemos a esperar, listos para movernos a cualquier parte.
//             }
//             break;
//     }
// }


// const float LEFT_CONTAINER_THETAS[3] = {-70, -62, -30};
// const float RIGHT_CONTAINER_THETAS[3] = {-30, -62, -70};

// const float LEFT_CONTAINER_ABOVE[3]  = {-25.0, 10.0, 10.0};
// const float LEFT_CONTAINER_LOWER[3]  = {-35.0, 18.0, 18.0};
// const float RIGHT_CONTAINER_ABOVE[3] = {25.0, -10.0, -10.0};
// const float RIGHT_CONTAINER_LOWER[3] = {35.0, -18.0, -18.0};















