import os
from ultralytics import YOLO
import cv2


# Ruta del modelo YOLOv8
model_path = 'yolov8n.pt'

# Cargar un modelo YOLOv8 preentrenado
model = YOLO(model_path)  # cargar un modelo preentrenado

# Umbral de confianza para las detecciones
threshold = 0.5

# Diccionario de nombres de clase para el conjunto de datos COCO (solo 'person')
class_name_dict = {0: 'person'}

# Índice de la cámara USB
camera_index = 1  # Cambia este valor al índice correcto de tu cámara USB
cap = cv2.VideoCapture(camera_index)
if not cap.isOpened():
    raise Exception("No se puede abrir la cámara web")

# Obtener las dimensiones del frame
ret, frame = cap.read()
if not ret:
    raise Exception("No se puede leer el frame de la cámara web")
H, W, _ = frame.shape

# Bucle principal para capturar y procesar frames de la cámara
while True:
    # Leer un frame de la cámara
    ret, frame = cap.read()
    if not ret:
        break

    # Realizar la detección con el modelo YOLOv8
    results = model(frame)[0]

    # Procesar cada detección
    for result in results.boxes.data.tolist():
        x1, y1, x2, y2, score, class_id = result

        # Filtrar solo la clase 'person' (ID 0) y aplicar el umbral de confianza
        if int(class_id) == 0 and score > threshold:
            # Dibujar un rectángulo alrededor de la detección
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 4)
            # Añadir una etiqueta de texto a la detección
            cv2.putText(frame, class_name_dict[int(class_id)].upper(), (int(x1), int(y1 - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0, 255, 0), 3, cv2.LINE_AA)

    # Mostrar el frame con las detecciones en una ventana
    cv2.imshow('Video', frame)

    # Salir si se presiona la tecla 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    # Leer el siguiente frame de la cámara
    ret, frame = cap.read()

# Liberar los recursos de captura y escritura de video
cap.release()
cv2.destroyAllWindows()
