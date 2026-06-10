#!/usr/bin/env python3
"""Put ONE Tello EDU into station (AP) mode so it joins your LAN/router.

Do this ONCE per drone. Repeat for the second drone.

Procedure (per drone):
  1. Power on the Tello EDU.
  2. Connect THIS computer's WiFi to the drone's own hotspot  TELLO-XXXXXX.
  3. Run:   python3 set_station_mode.py "<YOUR_WIFI_SSID>" "<YOUR_WIFI_PASSWORD>"
     (use ""  for the password on an open network)
  4. The drone replies 'ok', then reboots and connects to your router. Its own
     TELLO-XXXXXX hotspot disappears, so your computer drops that WiFi.
  5. Reconnect this computer to your router, then run find_tello_ips.py to get the
     drone's new IP and put it in config/swarm.yaml.

To undo (force a drone BACK to its own-hotspot mode): power it on and hold the
power button ~5 s until the LED blinks, which factory-resets the WiFi.

This talks raw Tello SDK over UDP — no djitellopy needed — so it's transparent and
dependency-free.
"""
import socket
import sys


TELLO_ADDR = ('192.168.10.1', 8889)   # the drone while on its own hotspot
LOCAL_PORT = 9000                       # we bind here; Tello replies to the source port


def send(sock, command, timeout=10.0):
    """Send one SDK command, wait for the drone's reply (or time out)."""
    print(f'  -> {command}')
    sock.sendto(command.encode('utf-8'), TELLO_ADDR)
    sock.settimeout(timeout)
    try:
        data, _ = sock.recvfrom(1024)
        reply = data.decode('utf-8', errors='ignore').strip()
        print(f'  <- {reply}')
        return reply
    except socket.timeout:
        print('  <- (no reply / timeout)')
        return None


def main():
    if len(sys.argv) < 2:
        print('usage: set_station_mode.py "<SSID>" "<PASSWORD>"   (password may be "")')
        sys.exit(1)
    ssid = sys.argv[1]
    password = sys.argv[2] if len(sys.argv) > 2 else ''

    print('Make sure this computer is connected to the drone\'s TELLO-XXXXXX WiFi.\n')

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('', LOCAL_PORT))

    # 1) enter SDK mode
    if send(sock, 'command') != 'ok':
        print('\nDrone did not enter SDK mode. Are you connected to its TELLO-XXXXXX WiFi?')
        sys.exit(1)

    # 2) tell it to join your router
    reply = send(sock, f'ap {ssid} {password}', timeout=15.0)
    if reply is not None and reply.lower().startswith('ok'):
        print(f'\nSUCCESS. The drone is rebooting and will join "{ssid}".')
        print('Reconnect this computer to your router, then run find_tello_ips.py.')
    else:
        print('\nThe drone did not confirm. Note: only Tello EDU supports station mode,')
        print('and the SSID/password must be correct. Try again from a fresh power-on.')
    sock.close()


if __name__ == '__main__':
    main()