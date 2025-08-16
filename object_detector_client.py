import cv2
import numpy as np
from ultralytics import YOLO
import socket
import time

class ObjectDetector:
    def __init__(self):
        self.model = YOLO('yolov8n.pt')  # Modelo ligero
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        
        # Configuración de coordenadas
        self.workspace_scale = 0.5  # Ajustar según calibración
        self.robot_z = -200         # Altura fija del efector
        
        # Configuración de socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            # Cambia 'localhost' por la IP del servidor si es necesario
            self.sock.connect(('127.0.0.1', 65432))  # Ejemplo para red local
            print("Conexión exitosa al servidor del robot delta")
        except ConnectionRefusedError:
            print("Error: El servidor no está activo o la IP es incorrecta")
            exit()

    def pixel_to_robot_coords(self, pixel_x, pixel_y):
        """Convierte coordenadas de píxeles a mm en el espacio del robot."""
        x_robot = (pixel_x - 640) * self.workspace_scale
        y_robot = (360 - pixel_y) * self.workspace_scale  # Invertir eje Y
        return x_robot, y_robot, self.robot_z

    def run(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break
            
            results = self.model(frame, verbose=False)
            for result in results:
                for box in result.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    centroid_x = (x1 + x2) // 2
                    centroid_y = (y1 + y2) // 2
                    
                    # Obtener coordenadas para el robot
                    x_robot, y_robot, z_robot = self.pixel_to_robot_coords(centroid_x, centroid_y)
                    
                    # Enviar coordenadas al servidor
                    coords_str = f"{x_robot:.1f},{y_robot:.1f},{z_robot:.1f}\n"
                    #self.sock.sendall(coords_str.encode())
                    
                    # Dibujar (opcional)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame, f"Robot: ({x_robot:.1f}, {y_robot:.1f})", 
                    (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

                    try:
                        print(f"Enviando: {coords_str.strip()}")  # Antes del sock.sendall
                        self.sock.sendall(coords_str.encode())
                        time.sleep(0.05)  # Pequeña pausa para no saturar
                    except Exception as e:
                        print(f"Error enviando datos: {e}")

            cv2.imshow('Detección', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        self.cap.release()
        cv2.destroyAllWindows()
        self.sock.close()

if __name__ == "__main__":
    detector = ObjectDetector()
    detector.run()