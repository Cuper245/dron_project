import cv2
import robomaster
from robomaster import robot
import time

robomaster.config.LOCAL_IP_STR = "192.168.10.2"

tl_drone = robot.Drone()
tl_drone.initialize()
# Open camera resources
tl_camera = tl_drone.camera
tl_camera.start_video_stream(display=False)
tl_camera.set_fps("high")
tl_camera.set_resolution("high")
tl_camera.set_bitrate(6)
# Visualize 300 frames
img = tl_camera.read_cv2_image()
cv2.imshow("Drone", img)
cv2.waitKey(1)
cv2.destroyAllWindows()
# Release camera resources
tl_camera.stop_video_stream()