// #include <HardwareSerial.h>

// void setup() {
//   Serial.begin(115200);
//   Serial2.begin(115200);  // Usando UART2 del ESP32 (ajusta según tu conexión)
  
//   Serial.println("Sistema listo para recibir coordenadas...");
// }

// void loop() {
//   if (Serial2.available() > 0) {
//     String data = Serial2.readStringUntil('\n');
//     data.trim();  // Eliminar espacios y caracteres no deseados
    
//     // Separar las coordenadas x e y
//     int commaIndex = data.indexOf(',');
//     if (commaIndex != -1) {
//       String xStr = data.substring(0, commaIndex);
//       String yStr = data.substring(commaIndex + 1);
      
//       int x = xStr.toInt();
//       int y = yStr.toInt();
      
//       // Mostrar en el monitor serial
//       Serial.print("Centroide recibido - X: ");
//       Serial.print(x);
//       Serial.print(", Y: ");
//       Serial.println(y);
//     }
//   }
  
//   delay(10);  // Pequeña pausa para evitar sobrecarga
// }



// void setup() {
//   // Inicializa comunicación serial por USB (para monitor serial)
//   Serial.begin(115200);
  
//   // Mensaje inicial
//   Serial.println("Sistema listo para recibir coordenadas...");
// }

// void loop() {
//   // Verifica si hay datos disponibles desde Python
//   if (Serial.available() > 0) {
//     // Lee hasta encontrar un salto de línea
//     String data = Serial.readStringUntil('\n');
//     data.trim();  // Elimina espacios y caracteres no deseados
    
//     // Busca la coma que separa las coordenadas
//     int commaIndex = data.indexOf(',');
    
//     if (commaIndex != -1) {
//       // Extrae coordenada X (antes de la coma)
//       String xStr = data.substring(0, commaIndex);
//       // Extrae coordenada Y (después de la coma)
//       String yStr = data.substring(commaIndex + 1);
      
//       // Convierte a números enteros
//       int x = xStr.toInt();
//       int y = yStr.toInt();
      
//       // Muestra en el monitor serial (Arduino IDE)
//       Serial.print("Centroide recibido - X: ");
//       Serial.print(x);
//       Serial.print(", Y: ");
//       Serial.println(y);
      
//       // Aquí puedes agregar lógica para usar las coordenadas:
//       // - Controlar motores
//       // - Mover un brazo robótico
//       // - Enviar a otra dispositivo, etc.
//     }
//   }
  
//   delay(10);  // Pequeña pausa para evitar sobrecarga
// }
#include <WiFi.h>
#include <WebServer.h>

// Configuración de red
const char* ssid = "Totalplay-A0A3";
const char* password = "Mustang1998";

// Variables para almacenar coordenadas
int lastX = 0;
int lastY = 0;
bool newData = false;

WebServer server(80);

void handleRoot() {
  String html = "<!DOCTYPE html><html><head>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>body {font-family: Arial; text-align: center; margin-top: 50px;}";
  html += ".data {font-size: 24px; color: #2c3e50; padding: 20px; background-color: #f8f9fa; border-radius: 10px; display: inline-block;}</style>";
  html += "<meta http-equiv='refresh' content='1'>";
  html += "</head><body>";
  html += "<h1>Sistema de Seguimiento YOLO</h1>";
  html += "<div class='data'>";
  html += "<p>Última coordenada recibida:</p>";
  html += "<p>X: " + String(lastX) + "</p>";
  html += "<p>Y: " + String(lastY) + "</p>";
  html += "</div></body></html>";
  
  server.send(200, "text/html", html);
}

void handleUpdate() {
  if (server.hasArg("x") && server.hasArg("y")) {
    lastX = server.arg("x").toInt();
    lastY = server.arg("y").toInt();
    newData = true;
    server.send(200, "text/plain", "OK");
    Serial.print("Datos actualizados - X: ");
    Serial.print(lastX);
    Serial.print(", Y: ");
    Serial.println(lastY);
  } else {
    server.send(400, "text/plain", "Bad Request");
  }
}

void handleNotFound() {
  server.send(404, "text/plain", "404: Not found");
}

void setup() {
  Serial.begin(115200);
  
  // Conectar a WiFi
  WiFi.begin(ssid, password);
  Serial.print("Conectando a WiFi");
  Serial.print("Listo para recibir coordenadas");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\nConectado!");
  Serial.print("Dirección IP: ");
  Serial.println(WiFi.localIP());
  
  // Configurar rutas
  server.on("/", handleRoot);
  server.on("/update", handleUpdate);
  server.onNotFound(handleNotFound);
  
  server.begin();
  Serial.println("Servidor HTTP iniciado");
}

void loop() {
  server.handleClient();
  
  if (newData) {
    newData = false;
    // Aquí puedes agregar lógica adicional cuando llegan nuevos datos
  }
  if (Serial.available() > 0) {
    // Lee hasta encontrar un salto de línea
    String data = Serial.readStringUntil('\n');
    data.trim();  // Elimina espacios y caracteres no deseados
    
    // Busca la coma que separa las coordenadas
    int commaIndex = data.indexOf(',');
    
    if (commaIndex != -1) {
      // Extrae coordenada X (antes de la coma)
      String xStr = data.substring(0, commaIndex);
      // Extrae coordenada Y (después de la coma)
      String yStr = data.substring(commaIndex + 1);
      
      // Convierte a números enteros
      int x = xStr.toInt();
      int y = yStr.toInt();
      
      // Muestra en el monitor serial (Arduino IDE)
      Serial.print("Centroide recibido - X: ");
      Serial.print(x);
      Serial.print(", Y: ");
      Serial.println(y);
     } 
  //delay(10);
}
}