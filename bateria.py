import robomaster
from robomaster import robot

robomaster.config.LOCAL_IP_STR = "192.168.10.2"

if __name__ == '__main__':
    tl_drone = robot.Drone()
    tl_drone.initialize()

    tl_flight = tl_drone.flight
    # Get battery status
    tl_battery = tl_drone.battery
    battery_info = tl_battery.get_battery()
    print("Drone battery soc: {0}".format(battery_info))

    # Close resources
    tl_drone.close()