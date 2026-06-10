from robomaster import robot
import robomaster

robomaster.config.LOCAL_IP_STR = '192.168.1.221'
robomaster.config.ROBOT_IP_STR = '192.168.1.73'

tl_drone = robot.Drone()
tl_drone.initialize(conn_type='sta')  # ← this line is missing in your file

battery = tl_drone.battery.get_battery()
print(f'Battery raw: {battery}')
version = tl_drone.get_sdk_version()
print(f'SDK version: {version}')
sn = tl_drone.get_sn()
print(f'SN: {sn}')
tl_drone.close()