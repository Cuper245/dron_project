import time
import cv2
import robomaster
from robomaster import robot

robomaster.config.LOCAL_IP_STR = "192.168.10.2"

if __name__ == '__main__':
    tl_drone = robot.Drone()
    tl_drone.initialize()

    try:
        tl_flight = tl_drone.flight
        tl_camera = tl_drone.camera

        # Takeoff
        tl_flight.takeoff().wait_for_completed()
        time.sleep(2)

        # Start camera
        tl_camera.start_video_stream(display=False)
        tl_camera.set_fps("high")
        tl_camera.set_resolution("high")
        tl_camera.set_bitrate(6)

        # Read one frame to get resolution
        img = tl_camera.read_cv2_image()
        height, width, _ = img.shape

        # Initialize VideoWriter
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter("video.mp4", fourcc, 20.0, (width, height))

        # Start motion
        tl_flight.rc(30, 20, 0, 50)

        start_time = time.time()

        while time.time() - start_time < 10:
            img = tl_camera.read_cv2_image()

            if img is not None:
                out.write(img)  # Save frame to video
                cv2.imshow("Drone", img)

            # Press q to stop early
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # Stop motion
        tl_flight.rc(0, 0, 0, 0)
        time.sleep(2)

        # Release video file
        out.release()

        # Stop camera
        tl_camera.stop_video_stream()
        cv2.destroyAllWindows()

        # Land
        tl_flight.land().wait_for_completed()

    finally:
        tl_drone.close()