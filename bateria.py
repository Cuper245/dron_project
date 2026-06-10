from robomaster import robot
import robomaster

robomaster.config.LOCAL_IP_STR = '192.168.1.221'
robomaster.config.ROBOT_IP_STR = '192.168.1.84'

tl_drone = robot.Drone()
tl_drone.initialize(conn_type='sta')

tl_battery = tl_drone.battery
battery_info = tl_battery.get_battery()
print("Drone battery soc: {0}".format(battery_info))

tl_drone.close()