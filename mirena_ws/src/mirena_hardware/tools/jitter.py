import cv2
import time
import numpy as np

# Pasar la ruta exacta de tu dispositivo udev
device_path = "/dev/video0"
cap = cv2.VideoCapture(device_path, cv2.CAP_V4L2)

if not cap.isOpened():
    print(f"Error: No se pudo abrir {device_path}")
    exit()

# Configuración nativa del sensor a 90 FPS
# Importante: V4L2 requiere que el formato se fuerce a MJPEG para alcanzar 90fps a esta resolución
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 640)
cap.set(cv2.CAP_PROP_FPS, 90)

# Reducir el buffer de la cola de V4L2 al mínimo para evitar frames antiguos
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

tiempos = []

print("Estabilizando flujo de video en Ubuntu...")
for _ in range(30):
    cap.grab()

print("Capturando 500 marcas de tiempo...")
for _ in range(500):
    ret = cap.grab()
    if ret:
        # Instante en que el kernel de Linux libera el frame del buffer USB
        t_llegada = time.perf_counter()
        tiempos.append(t_llegada)

cap.release()

# Calcular deltas en milisegundos
intervalos = np.diff(tiempos) * 1000
jitter_software = np.std(intervalos)
intervalo_teorico = 1000 / 90

print(f"\n--- RESULTADOS EN UBUNTU ({device_path}) ---")
print(f"Intervalo Teórico Mínimo: {intervalo_teorico:.2f} ms")
print(f"Intervalo Promedio Real:  {np.mean(intervalos):.2f} ms")
print(f"Jitter Real del Sistema:   ±{jitter_software:.4f} ms")
print(f"Intervalo Mínimo:         {np.min(intervalos):.2f} ms")
print(f"Intervalo Máximo:         {np.max(intervalos):.2f} ms")