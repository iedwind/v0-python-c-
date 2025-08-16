import cv2
from ultralytics import YOLO

# 1. Cargar el modelo YOLOv8 preentrenado (se descarga automáticamente si no está local)
model = YOLO("yolov8s.pt")  # Cambia a "yolov8s.pt" si quieres más precisión

# 2. Abrir el video (asegúrate de que "video.mp4" está en el mismo directorio)
#video_path = video(0)
cap = cv2.VideoCapture(0)

# 3. Procesar el video frame por frame
while cap.isOpened():
    success, frame = cap.read()
    
    if not success:
        break  # Fin del video
    
    # Ejecutar YOLOv8 en el frame (inferencia)
    results = model(frame, conf=0.58)  # conf = umbral de confianza (ajústalo)
    
    # Dibujar las detecciones en el frame
    annotated_frame = results[0].plot()
    
    # Mostrar el frame procesado
    cv2.imshow("YOLOv8", annotated_frame)
    
    # Salir con 'q'
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# # Liberar recursos
# cap.release()
# cv2.destroyAllWindows()
# import cv2
# from ultralytics import YOLO
# import time

# # 1. Cargar el modelo YOLOv8
# model = YOLO("yolov8n.pt")  # Usa "yolov8s.pt" para mayor precisión

# # 2. Abrir el video de entrada
# video_path = "video.mp4"
# cap = cv2.VideoCapture(video_path)

# # 3. Configurar el video de salida (guardar resultado)
# output_path = "output_video3.mp4"
# fps = 60  # Ajusta los FPS para hacerlo más lento (ej: 10 FPS)
# frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
# frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
# fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Codec para MP4
# out = cv2.VideoWriter(output_path, fourcc, fps, (frame_width, frame_height))

# # 4. Procesar el video frame por frame
# while cap.isOpened():
#     success, frame = cap.read()
    
#     if not success:
#         break  # Fin del video
    
#     # Ejecutar YOLOv8 en el frame
#     results = model(frame, conf=0.5)  # conf = umbral de confianza
    
#     # Dibujar las detecciones en el frame
#     annotated_frame = results[0].plot()
    
#     # Guardar el frame procesado en el video de salida
#     out.write(annotated_frame)
    
#     # Mostrar el frame en una ventana (a velocidad reducida)
#     cv2.imshow("YOLOv8 - Clasificación de basura (Lento)", annotated_frame)
    
#     # Controlar la velocidad con cv2.waitKey() y time.sleep()
#     delay = int(1000 / fps)  # Convertir FPS a milisegundos (ej: 1000/10 = 100ms por frame)
#     if cv2.waitKey(delay) & 0xFF == ord("q"):
#         break

# # 5. Liberar recursos
# cap.release()
# out.release()  # Guardar el video
# cv2.destroyAllWindows()

# print(f"¡Video procesado guardado como {output_path}! 🎥")