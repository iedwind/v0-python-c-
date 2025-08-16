# import numpy as np
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
# import math
# import socket
# import threading

# class DeltaRobotVisualizer:
#     def __init__(self):
#         self.L_arm = 200
#         self.L_rod = 400
#         self.R_base = 100
#         self.R_effector = 50
#         self.fig = plt.figure(figsize=(10, 8))
#         self.ax = self.fig.add_subplot(111, projection='3d')
#         self.setup_plot()
#         self.current_coords = (0, 0, -300)  # Posición inicial
#         plt.ion()  # Modo interactivo

#     def setup_plot(self):
#         self.ax.set_xlim([-300, 300])
#         self.ax.set_ylim([-300, 300])
#         self.ax.set_zlim([-400, 100])
#         self.ax.set_xlabel('X (mm)')
#         self.ax.set_ylabel('Y (mm)')
#         self.ax.set_zlabel('Z (mm)')

#     def inverse_kinematics(self, x, y, z):
#         theta = []
#         for i in range(3):
#             angle = 2 * np.pi / 3 * i
#             Bx = self.R_base * np.cos(angle)
#             By = self.R_base * np.sin(angle)
#             Px = x + self.R_effector * np.cos(angle)
#             Py = y + self.R_effector * np.sin(angle)
#             Pz = z
            
#             D = (Px - Bx)**2 + (Py - By)**2 + Pz**2
#             a = (D + self.L_arm**2 - self.L_rod**2) / (2 * self.L_arm)
#             b = np.sqrt((Px - Bx)**2 + (Py - By)**2)
#             c = Pz
            
#             theta_i = np.arctan2(c, b) - np.arctan2(a, np.sqrt(b**2 + c**2 - a**2))
#             theta.append(np.degrees(theta_i))
#         return theta

#     def update_plot(self, x, y, z):
#         self.ax.clear()
#         self.setup_plot()
#         theta = self.inverse_kinematics(x, y, z)
        
#         # Dibujar base
#         base_points = []
#         for i in range(3):
#             angle = 2 * np.pi / 3 * i
#             bx = self.R_base * np.cos(angle)
#             by = self.R_base * np.sin(angle)
#             base_points.append([bx, by, 0])
#         base_points.append(base_points[0])
#         base_points = np.array(base_points)
#         self.ax.plot(base_points[:, 0], base_points[:, 1], base_points[:, 2], 'b-', linewidth=2)
        
#         # Dibujar brazos y bielas
#         for i in range(3):
#             angle = 2 * np.pi / 3 * i
#             Bx = self.R_base * np.cos(angle)
#             By = self.R_base * np.sin(angle)
#             Bz = 0
            
#             Ax = Bx + self.L_arm * np.cos(np.radians(theta[i])) * np.cos(angle)
#             Ay = By + self.L_arm * np.cos(np.radians(theta[i])) * np.sin(angle)
#             Az = Bz + self.L_arm * np.sin(np.radians(theta[i]))
            
#             Px = x + self.R_effector * np.cos(angle)
#             Py = y + self.R_effector * np.sin(angle)
#             Pz = z
            
#             self.ax.plot([Bx, Ax], [By, Ay], [Bz, Az], 'r-', linewidth=3)
#             self.ax.plot([Ax, Px], [Ay, Py], [Az, Pz], 'g-', linewidth=3)
        
#         self.ax.scatter(x, y, z, color='purple', s=100)
#         self.ax.set_title(f'Posición: ({x:.1f}, {y:.1f}, {z:.1f}) mm\nÁngulos: {theta}°')
#         plt.draw()
#         plt.pause(0.01)

#     def start_server(self):
#         with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
#             s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
#             s.bind(('localhost', 65432))
#             s.listen()
#             print("Servidor escuchando en 127.0.0.1:65432... (Presiona Ctrl+C para detener)")
#             while True:  # Bucle infinito para mantener el servidor activo
#                 conn, addr = s.accept()
#                 print(f"\nConexión aceptada desde {addr}")  # Mensaje al recibir conexión
#                 with conn:
#                     try:
#                         while True:
#                             data = conn.recv(1024).decode()
#                             if not data:
#                                 break
#                             x, y, z = map(float, data.split(','))
#                             self.update_plot(x, y, z)
#                     except Exception as e:
#                         print(f"Error en la conexión: {e}")


# if __name__ == "__main__":
#     visualizer = DeltaRobotVisualizer()
#     # Iniciar servidor en un hilo separado
#     server_thread = threading.Thread(target=visualizer.start_server)
#     server_thread.daemon = True
#     server_thread.start()
    
#     # Mantener la ventana abierta
#     try:
#         plt.show()
#     except KeyboardInterrupt:
#         print("Cerrando servidor...")

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import socket
import threading
import queue
import math

class DeltaRobotVisualizer:
    def __init__(self):
        # Parámetros del robot (en mm)
        self.L_arm = 200      # Longitud del brazo
        self.L_rod = 400      # Longitud de la biela
        self.R_base = 100     # Radio de la base
        self.R_effector = 50  # Radio del efector
        
        # Configuración de la gráfica
        self.fig = plt.figure(figsize=(10, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.setup_plot()
        
        # Comunicación entre hilos
        self.update_queue = queue.Queue()
        self.running = True
        
        # Elementos gráficos (inicializados vacíos)
        self.base_lines, = self.ax.plot([], [], [], 'b-', linewidth=2)
        self.arm_lines = [self.ax.plot([], [], [], 'r-', linewidth=3)[0] for _ in range(3)]
        self.rod_lines = [self.ax.plot([], [], [], 'g-', linewidth=3)[0] for _ in range(3)]
        self.efector_dot = self.ax.scatter([0], [0], [-300], color='purple', s=100)
        
        # Iniciar servidor en segundo plano
        self.server_thread = threading.Thread(target=self.start_server, daemon=True)
        self.server_thread.start()
        
        # Configurar animación
        self.ani = FuncAnimation(self.fig, self.update_animation, interval=50, blit=False)

    def setup_plot(self):
        self.ax.set_xlim([-600, 600])
        self.ax.set_ylim([-600, 600])
        self.ax.set_zlim([-400, 100])
        self.ax.set_xlabel('X (mm)')
        self.ax.set_ylabel('Y (mm)')
        self.ax.set_zlabel('Z (mm)')
        self.ax.set_title('Robot Delta - Esperando datos...')

    def start_server(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind(('localhost', 65432))
            s.listen()
            print("🟢 Servidor Delta activo en localhost:65432")
            
            while self.running:
                conn, addr = s.accept()
                with conn:
                    print(f"🔗 Cliente conectado: {addr}")
                    while self.running:
                        try:
                            data = conn.recv(1024).decode().strip()
                            if not data:
                                break
                            
                            # Validar formato "x,y,z"
                            if data.count(',') == 2:
                                x, y, z = map(float, data.split(','))
                                self.update_queue.put((x, y, z))
                        except (ValueError, ConnectionResetError) as e:
                            print(f"⚠️ Error en datos: {e}")
                            break

    def calculate_angles(self, x, y, z):
        theta = []
        for i in range(3):
            angle = 2 * math.pi / 3 * i
            Bx = self.R_base * math.cos(angle)
            By = self.R_base * math.sin(angle)
            Px = x + self.R_effector * math.cos(angle)
            Py = y + self.R_effector * math.sin(angle)
            Pz = z
            
            D = (Px - Bx)**2 + (Py - By)**2 + Pz**2
            a = (D + self.L_arm**2 - self.L_rod**2) / (2 * self.L_arm)
            b = math.sqrt((Px - Bx)**2 + (Py - By)**2)
            c = Pz
            
            theta_i = math.atan2(c, b) - math.atan2(a, math.sqrt(b**2 + c**2 - a**2))
            theta.append(math.degrees(theta_i))
        return theta

    def update_animation(self, frame):
        try:
            while True:  # Procesa todos los datos pendientes
                x, y, z = self.update_queue.get_nowait()
                
                # Actualizar efector
                self.efector_dot._offsets3d = ([x], [y], [z])
                
                # Calcular cinemática inversa
                theta = self.calculate_angles(x, y, z)
                
                # Actualizar base (triángulo)
                base_points = []
                for i in range(4):
                    angle = 2 * math.pi / 3 * (i % 3)
                    base_points.append([
                        self.R_base * math.cos(angle),
                        self.R_base * math.sin(angle),
                        0
                    ])
                base_points = np.array(base_points).T
                self.base_lines.set_data(base_points[0], base_points[1])
                self.base_lines.set_3d_properties(base_points[2])
                
                # Actualizar brazos y bielas
                for i in range(3):
                    angle = 2 * math.pi / 3 * i
                    Bx = self.R_base * math.cos(angle)
                    By = self.R_base * math.sin(angle)
                    
                    # Punto de articulación
                    Ax = Bx + self.L_arm * math.cos(math.radians(theta[i])) * math.cos(angle)
                    Ay = By + self.L_arm * math.cos(math.radians(theta[i])) * math.sin(angle)
                    Az = 0 + self.L_arm * math.sin(math.radians(theta[i]))
                    
                    # Punto del efector
                    Px = x + self.R_effector * math.cos(angle)
                    Py = y + self.R_effector * math.sin(angle)
                    Pz = z
                    
                    # Actualizar líneas
                    self.arm_lines[i].set_data([Bx, Ax], [By, Ay])
                    self.arm_lines[i].set_3d_properties([0, Az])
                    
                    self.rod_lines[i].set_data([Ax, Px], [Ay, Py])
                    self.rod_lines[i].set_3d_properties([Az, Pz])
                
                self.ax.set_title(f'Robot Delta - Posición: ({x:.1f}, {y:.1f}, {z:.1f}) mm')
                
        except queue.Empty:
            pass
        
        return [self.base_lines, *self.arm_lines, *self.rod_lines, self.efector_dot]

    def run(self):
        try:
            plt.tight_layout()
            plt.show()
        except KeyboardInterrupt:
            self.running = False
            print("\n👋 Programa terminado correctamente")
        finally:
            plt.close('all')

if __name__ == "__main__":
    print("Iniciando visualizador del Robot Delta...")
    robot = DeltaRobotVisualizer()
    robot.run()