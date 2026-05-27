import sys
import time
import cv2
from queue import Empty
import robomaster
from robomaster import robot
import cv2.aruco as aruco

robomaster.config.LOCAL_IP_STR = "192.168.10.2"

if sys.platform == "win32":
    sys.stdout.reconfigure(encoding="utf-8")

print("Iniciando drone...")
tl_drone = robot.Drone()
tl_drone.initialize()
print("Drone inicializado.")

tl_camera = tl_drone.camera
tl_camera.start_video_stream(display=False)
tl_camera.set_resolution("high")
tl_camera.set_down_vision(1)
time.sleep(2)

print("Esperando primer frame...")
frame = None
while frame is None:
    try:
        frame = tl_camera.read_cv2_image(timeout=5)
    except Empty:
        time.sleep(0.2)

print(f"Resolucion: {frame.shape[1]}x{frame.shape[0]}")
print("Q = salir\n")

try:
    while True:
        try:
            frame = tl_camera.read_cv2_image(timeout=1)
        except Empty:
            continue
        if frame is None:
            continue

        # add inside the while loop, after reading the frame:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        detector = aruco.ArucoDetector(dictionary, aruco.DetectorParameters())
        corners, ids, _ = detector.detectMarkers(gray)
        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)

        cv2.imshow("Camara Inferior", frame)



        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

finally:
    cv2.destroyAllWindows()
    tl_camera.stop_video_stream()
    tl_drone.close()
    print("Cerrado.")