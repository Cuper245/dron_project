import cv2
import cv2.aruco as aruco

dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(dictionary, parameters)

cap = cv2.VideoCapture(0)  # laptop webcam

while True:
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = detector.detectMarkers(gray)

    if ids is not None:
        aruco.drawDetectedMarkers(frame, corners, ids)
        # compute center of each detected marker
        for i, corner in enumerate(corners):
            cx = int(corner[0][:, 0].mean())
            cy = int(corner[0][:, 1].mean())
            print(f"Marker {ids[i][0]}: center=({cx},{cy})")

    cv2.imshow("ArUco Test", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break