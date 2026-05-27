import robomaster
import time 
from robomaster import robot

robomaster.config.LOCAL_IP_STR = "192.168.10.2"

if __name__ == '__main__':
    tl_drone = robot.Drone()
    tl_drone.initialize()

    tl_flight = tl_drone.flight

    # Set the QUAV to takeoff
    tl_flight.takeoff().wait_for_completed()
    time.sleep(2)

    tl_flight.up(distance=50).wait_for_completed()

    tl_flight.forward(distance=50).wait_for_completed()
    tl_flight.rotate(angle=-45).wait_for_completed()
    
    tl_flight.forward(distance=100).wait_for_completed()
    tl_flight.rotate(angle=135).wait_for_completed()

    tl_flight.forward(distance=80).wait_for_completed()
    tl_flight.rotate(angle=135).wait_for_completed()

    tl_flight.forward(distance=100).wait_for_completed()

    tl_flight.down(distance=50).wait_for_completed()

    # Set the QUAV to land
    tl_flight.land().wait_for_completed()

    # Close resources
    tl_drone.close()