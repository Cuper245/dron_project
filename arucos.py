import cv2
import cv2.aruco as aruco

dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

for marker_id in [0, 1]:  # one per station
    img = aruco.generateImageMarker(dictionary, marker_id, 300)
    cv2.imwrite(f"station_{marker_id}.png", img)