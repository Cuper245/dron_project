# connect_to_wifi.py — run this ONCE while connected to TELLO_FE193A
import socket
import time

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(b'command', ('192.168.10.1', 8889))
time.sleep(1)
# send wifi credentials
cmd = f'ap Lab_Robotica coke.fanta.sprite'.encode()
sock.sendto(cmd, ('192.168.10.1', 8889))
time.sleep(3)
sock.close()
print("Done — drone will reboot and connect to Lab_Robotica")