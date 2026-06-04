"""
mission.py

State machine for the autonomous delivery mission.
All drone-hardware calls are wrapped in methods so they can be
stubbed out for offline testing (set OFFLINE_MODE = True below).
"""
import os
import time
import numpy as np
from enum import Enum
from pd_controller import PDController
from vicon_subscriber import start_vicon_thread  # swap for start_vicon_thread in lab
from queue import Empty
import cv2



# ── Set to False when running on real drone ───────────────────────────────────

OFFLINE_MODE = os.getenv("OFFLINE_MODE", "false").lower() == "true"

# ── Cruise altitude for Phase 1 flight (metres) ───────────────────────────────
CRUISE_ALTITUDE = 0.7

# ── How close is "close enough" to switch to Phase 2 (metres) ────────────────
PHASE1_THRESHOLD = 0.15

# ── How centred is "centred enough" to land (pixels) ─────────────────────────
CENTRE_PIXEL_THRESHOLD = 15


class MissionState(Enum):
    IDLE       = "idle"
    SCANNING   = "scanning"
    FLYING     = "flying_to_station"
    LANDING    = "precision_landing"
    LANDED     = "landed"
    ABORT      = "abort"


class MissionController:
    def __init__(self):
        self.state       = MissionState.IDLE
        self.station_map = {}          # {aruco_id: (x, y, z)}
        self.target_id   = None

        # shared dict updated by Vicon thread
        self.vicon = {}
        start_vicon_thread(self.vicon)   # ← swap for start_vicon_thread(self.vicon) in lab

        self.pd = PDController(
            Kp_xy=0.30, Kd_xy=0.10,
            Kp_z=0.50,  Kd_z=0.15,
        )

        # drone SDK objects — set by init_drone()
        self._tl_drone  = None
        self._tl_flight = None
        self._tl_camera = None

        if not OFFLINE_MODE:
            self._init_drone()

    # ── Hardware init ─────────────────────────────────────────────────────────
    def _init_drone(self):
        import robomaster
        from robomaster import robot

        import cv2.aruco as aruco

        robomaster.config.LOCAL_IP_STR = "192.168.10.2"
        self._tl_drone = robot.Drone()
        self._tl_drone.initialize()
        self._tl_flight = self._tl_drone.flight
        self._tl_camera = self._tl_drone.camera
        self._tl_camera.start_video_stream(display=False)
        self._tl_camera.set_resolution("high")  # ← from camaraAbajo.py
        self._tl_camera.set_down_vision(1)       # ← switches to downward camera
        time.sleep(2)                            # ← let camera stabilize
        print("[mission] Drone initialised")

    # ── Public API (called by Flask server) ───────────────────────────────────
    @property
    def drone_pos(self):
        return self.vicon.get("drone_pos", (0.0, 0.0, 0.0))

    @property
    def drone_yaw(self):
        return self.vicon.get("drone_yaw", 0.0)

    def dispatch(self, station_id):
        if self.state != MissionState.IDLE:
            return {"error": f"busy — current state: {self.state.value}"}
        if station_id not in self.station_map:
            return {"error": "station not mapped yet"}
        self.target_id = station_id
        self.pd.reset()
        self.state = MissionState.FLYING
        print(f"[mission] Dispatched to station {station_id}")
        return {"status": "dispatched", "target": station_id}

    def abort(self):
        self.state = MissionState.ABORT
        self._send_rc(0, 0, 0, 0)
        print("[mission] ABORT — hovering")

    # ── Main loop tick ────────────────────────────────────────────────────────
    def update(self):
        """Call this in a loop at ~20 Hz."""
        if self.state == MissionState.IDLE:
            pass

        elif self.state == MissionState.SCANNING:
            self._do_scan_tick()

        elif self.state == MissionState.FLYING:
            target_xyz = self.station_map[self.target_id]
            # Hold cruise altitude until we are above the station
            target_with_altitude = (target_xyz[0], target_xyz[1], CRUISE_ALTITUDE)
            lr, fb, ud, yaw = self.pd.compute(
                self.drone_pos, target_with_altitude, self.drone_yaw
            )
            self._send_rc(lr, fb, ud, yaw)
            if self.pd.within_threshold(self.drone_pos, target_with_altitude, PHASE1_THRESHOLD):
                print("[mission] Phase 1 complete — switching to ArUco landing")
                self.pd.reset()
                self.state = MissionState.LANDING

        elif self.state == MissionState.LANDING:
            detected_id, offset_px = self._detect_aruco_from_below()
            if detected_id is None:
                # Lost the marker — hover and wait
                self._send_rc(0, 0, 0, 0)
                return
            if detected_id != self.target_id:
                print(f"[mission] ABORT — expected ID {self.target_id}, saw {detected_id}")
                self.state = MissionState.ABORT
                return
            self._center_over_marker(offset_px)
            if self._is_centred(offset_px):
                self._land()
                self.state = MissionState.LANDED
                print(f"[mission] Landed at station {self.target_id}")

        elif self.state == MissionState.ABORT:
            self._send_rc(0, 0, 0, 0)

    # ── Scan flight ───────────────────────────────────────────────────────────
    def start_scan(self):
        if self.state != MissionState.IDLE:
            return {"error": "not idle"}
        self.station_map = {}
        self.state = MissionState.SCANNING
        return {"status": "scanning"}

    def _do_scan_tick(self):
        """
        TODO (lab): implement lawnmower waypoint scan.
        For now: detect ArUco from current position and stamp with Vicon coords.
        """
        detected_id, _ = self._detect_aruco_from_below()
        if detected_id is not None and detected_id not in self.station_map:
            pos = self.drone_pos
            self.station_map[detected_id] = pos
            print(f"[mission] Mapped station {detected_id} at {pos}")

    # ── ArUco detection (Phase 2) ─────────────────────────────────────────────
    def _detect_aruco_from_below(self):
        if OFFLINE_MODE:
            return self.target_id, (5, -3)

        # read latest frame from downward camera
        try:
            frame = self._tl_camera.read_cv2_image(timeout=1)  # same as camaraAbajo.py
        except Empty:
            return None, None
        if frame is None:
            return None, None

        h, w = frame.shape[:2]
        frame_cx, frame_cy = w // 2, h // 2

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        detector   = aruco.ArucoDetector(dictionary, aruco.DetectorParameters())
        corners, ids, _ = detector.detectMarkers(gray)

        if ids is None:
            return None, None

        for i, marker_id in enumerate(ids.flatten()):
            cx = int(corners[i][0][:, 0].mean())
            cy = int(corners[i][0][:, 1].mean())
            return int(marker_id), (cx - frame_cx, cy - frame_cy)

        return None, None

    # ── Phase 2 centering ─────────────────────────────────────────────────────
    def _center_over_marker(self, offset_px):
        """
        Convert pixel offset to small rc() corrections.
        These gains will need tuning in the lab.
        """
        if offset_px is None:
            self._send_rc(0, 0, 0, 0)
            return

        Kp_px = 0.08   # TODO: tune in lab
        ox, oy = offset_px

        # Tello downward camera: +x pixel → drone needs to move right (positive lr)
        #                        +y pixel → drone needs to move forward (positive fb)
        # Adjust signs if the camera is rotated relative to the drone body.
        lr = int(np.clip( Kp_px * ox, -20, 20))
        fb = int(np.clip(-Kp_px * oy, -20, 20))
        self._send_rc(lr, fb, 0, 0)

    def _is_centred(self, offset_px):
        if offset_px is None:
            return False
        return (abs(offset_px[0]) < CENTRE_PIXEL_THRESHOLD and
                abs(offset_px[1]) < CENTRE_PIXEL_THRESHOLD)

    # ── Low-level drone commands ──────────────────────────────────────────────
    def _send_rc(self, lr, fb, ud, yaw):
        if OFFLINE_MODE:
            print(f"[rc stub] lr={lr:4d}  fb={fb:4d}  ud={ud:4d}  yaw={yaw:4d}")
            return
        self._tl_flight.rc(lr, fb, ud, yaw)

    def _land(self):
        if OFFLINE_MODE:
            print("[land stub] landing")
            return
        self._send_rc(0, 0, 0, 0)
        time.sleep(0.5)
        self._tl_flight.land().wait_for_completed()

    def takeoff(self):
        if OFFLINE_MODE:
            print("[takeoff stub]")
            return
        self._tl_flight.takeoff().wait_for_completed()
        time.sleep(2)

    def close(self):
        if OFFLINE_MODE:
            return
        self._tl_camera.stop_video_stream()
        self._tl_drone.close()


# ── Quick offline integration test ───────────────────────────────────────────
if __name__ == "__main__":
    print("\n── Offline mission integration test ──\n")

    mc = MissionController()

    # Simulate Vicon placing drone at origin
    mc.vicon["drone_pos"] = (0.0, 0.0, 0.7)
    mc.vicon["drone_yaw"] = 0.0
    mc.vicon["vicon_ok"]  = True

    # Manually add a fake station (normally filled by scan)
    mc.station_map[0] = (1.2, 0.4, 0.0)

    print("State:", mc.state.value)
    result = mc.dispatch(0)
    print("Dispatch result:", result)
    print("State:", mc.state.value)

    print("\nSimulating 5 update() ticks toward station —")
    for i in range(5):
        mc.update()

    print("\nSimulating arrival (move drone to near target) —")
    mc.vicon["drone_pos"] = (1.19, 0.39, 0.7)
    mc.update()
    print("State after arrival:", mc.state.value)

    print("\nSimulating landing phase —")
    mc.update()   # should see ArUco stub and start centering
    mc.vicon["drone_pos"] = (1.20, 0.40, 0.1)
    mc.update()
    print("Final state:", mc.state.value)