import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math

# Parámetros del robot (ajustables)
L_arm = 200    # Longitud del brazo (mm)
L_rod = 400    # Longitud de la biela (mm)
R_base = 100   # Radio de la base (mm)
R_effector = 50 # Radio del efector (mm)

# Cinemática inversa corregida
# def delta_inverse_kinematics(x, y, z):
#     theta = []
#     for i in range(3):
#         angle = 2 * np.pi / 3 * i  # 0°, 120°, 240°
#         Bx = R_base * np.cos(angle)
#         By = R_base * np.sin(angle)
#         Px = x + R_effector * np.cos(angle)
#         Py = y + R_effector * np.sin(angle)
#         Pz = z
        
#         D = (Px - Bx)**2 + (Py - By)**2 + Pz**2
#         a = (D + L_arm**2 - L_rod**2) / (2 * L_arm)
#         b = np.sqrt((Px - Bx)**2 + (Py - By)**2)
#         c = Pz
        
#         theta_i = np.arctan2(c, b) - np.arctan2(a, np.sqrt(b**2 + c**2 - a**2))
#         theta.append(np.degrees(theta_i))
    
#     return theta
def delta_inverse_kinematics(x, y, z):
    theta = []
    for i in range(3):
        angle = 2 * np.pi / 3 * i
        Bx = R_base * np.cos(angle)
        By = R_base * np.sin(angle)
        Px = x + R_effector * np.cos(angle)
        Py = y + R_effector * np.sin(angle)
        Pz = z
        
        # Cálculo intermedio
        D = (Px - Bx)**2 + (Py - By)**2 + Pz**2
        a = (D + L_arm**2 - L_rod**2) / (2 * L_arm)
        b = np.sqrt((Px - Bx)**2 + (Py - By)**2)
        c = Pz
        
        # Validación de posición alcanzable
        discriminant = b**2 + c**2 - a**2
        if discriminant < 0:
            theta.append(float('nan'))
            continue
            
        theta_i = np.arctan2(c, b) - np.arctan2(a, np.sqrt(discriminant))
        theta.append(np.degrees(theta_i))
    
    return theta
# Visualización 3D del robot
def plot_delta_robot(x, y, z, theta_degrees):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Convertir ángulos a radianes
    theta_radians = np.radians(theta_degrees)
    
    # Dibujar base (triángulo equilátero)
    base_points = []
    for i in range(3):
        angle = 2 * np.pi / 3 * i
        bx = R_base * np.cos(angle)
        by = R_base * np.sin(angle)
        base_points.append([bx, by, 0])
    base_points.append(base_points[0])  # Cerrar el triángulo
    base_points = np.array(base_points)
    ax.plot(base_points[:, 0], base_points[:, 1], base_points[:, 2], 'b-', linewidth=2, label='Base')
    
    # Dibujar brazos y bielas
    for i in range(3):
        angle = 2 * np.pi / 3 * i
        # Punto de anclaje en la base
        Bx = R_base * np.cos(angle)
        By = R_base * np.sin(angle)
        Bz = 0
        
        # Punto de articulación del brazo
        Ax = Bx + L_arm * np.cos(theta_radians[i]) * np.cos(angle)
        Ay = By + L_arm * np.cos(theta_radians[i]) * np.sin(angle)
        Az = Bz + L_arm * np.sin(theta_radians[i])
        
        # Punto de anclaje en el efector
        Px = x + R_effector * np.cos(angle)
        Py = y + R_effector * np.sin(angle)
        Pz = z
        
        # Dibujar brazo (B -> A)
        ax.plot([Bx, Ax], [By, Ay], [Bz, Az], 'r-', linewidth=3, label='Brazo' if i == 0 else "")
        # Dibujar biela (A -> P)
        ax.plot([Ax, Px], [Ay, Py], [Az, Pz], 'g-', linewidth=3, label='Biela' if i == 0 else "")
    
    # Dibujar efector
    ax.scatter(x, y, z, color='purple', s=100, label='Efector')
    
    # Ajustes de la visualización
    ax.set_xlim([-300, 300])
    ax.set_ylim([-300, 300])
    ax.set_zlim([-400, 100])
    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.set_zlabel('Z (mm)')
    ax.set_title(f'Robot Delta - Posición: ({x}, {y}, {z}) mm\nÁngulos: {theta_degrees}°')
    ax.legend()
    plt.tight_layout()
    plt.show()

# Ejemplo de uso
if __name__ == "__main__":
    # Posición objetivo (ajustar aquí)
    x, y, z = 0, 0, -300  # Z debe ser negativo
    
    # Calcular ángulos
    theta1, theta2, theta3 = delta_inverse_kinematics(x, y, z)
    print(f"Ángulos calculados:")
    print(f"θ1 = {theta1:.2f}°")
    print(f"θ2 = {theta2:.2f}°")
    print(f"θ3 = {theta3:.2f}°")
    
    # Verificar si la posición es alcanzable
    if any(math.isnan(theta) for theta in [theta1, theta2, theta3]):
        print("¡Error: Posición no alcanzable!")
    else:
        # Generar visualización
        plot_delta_robot(x, y, z, [theta1, theta2, theta3])