import time
import cv2
import robomaster
from robomaster import robot
import numpy as np

robomaster.config.LOCAL_IP_STR = "192.168.10.2"

if __name__ == '__main__':
    tl_drone = robot.Drone()
    tl_drone.initialize()
    tl_flight = tl_drone.flight
    # Get battery status
    tl_battery = tl_drone.battery
    battery_info = tl_battery.get_battery()
    print("Drone battery soc: {0}".format(battery_info))

    #Setting up camera
    tl_camera = tl_drone.camera.EPCamera
    tl_camera.start_video_stream(display=False)

    #Set color range for green detection
    lower = np.array([50, 100, 100])
    upper = np.array([70, 255, 255])

    try:
        while True:
            # Read the image from the camera
            frame = tl_camera.read_cv2_image(strategy="newest", timeout=5)
            # Frame size 480x360
            frame = cv2.resize(frame, (480, 360))
            # Convert to HSV color space
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            # Create a mask for green color
            mask = cv2.inRange(hsv, lower, upper)
            # Find contours in the mask
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE,
            cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                # Get the largest contour
                largest = max(contours, key=cv2.contourArea)
                # Parameters of rectangle of largest contour
                x, y, w, h = cv2.boundingRect(largest)
                # Center x coordinate of the object
                cx = x + w // 2
                # Center y coordinate of the object
                cy = y + h // 2
                # Area of the bounding rectangle
                area = w * h
                #obtain region of interest
                object_region = hsv[y:y+h, x:x+w]
                avg_color_rgb = cv2.mean(object_region)[:3]
                print(f"RGB color: R={avg_color_rgb[2]:.2f}, G={avg_color_rgb[1]:.2f}, B={avg_color_rgb[0]:.2f}")

                # Draw on screen
                # Draw rectangle around the detected object
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                # Draw circle at the center of the object
                cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)
                # Write color detected on the screen
                cv2.putText(frame, "Green detected", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            else:
                print("[INFO] Green not detected.")

            # Display the frame until 'q' is pressed
            cv2.imshow("Camera", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        print("Remote stop - L A N D I N G ...")
        #End flight
    finally:
        tl_camera.stop_video_stream()
        tl_drone.close()
        cv2.destroyAllWindows()
    # Close resources
    tl_drone.close()