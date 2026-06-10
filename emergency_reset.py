# emergency_reset.py
import robomaster
from robomaster import robot
import time

robomaster.config.LOCAL_IP_STR = '192.168.1.221'
robomaster.config.ROBOT_IP_STR = '192.168.1.84'

tl_drone = robot.Drone()
tl_drone.initialize(conn_type='sta')
time.sleep(2)
tl_drone.flight.emergency()  # clears any stuck state
time.sleep(1)
tl_drone.close()