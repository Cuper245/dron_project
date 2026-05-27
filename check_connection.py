import robomaster
from robomaster import robot

robomaster.config.LOCAL_IP_STR = "192.168.10.2"

if __name__ == '__main__':
    tl_drone = robot.Drone()
    tl_drone.initialize()

    drone_version = tl_drone.get_sdk_version()
    print("Drone sdk version:", drone_version)

    sn = tl_drone.get_sn()
    print("Drone SN:", sn)

    tl_drone.close()