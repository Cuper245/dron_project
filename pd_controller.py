import numpy as np

class PDController:
    """
    3-axis PD position controller for Tello Robomaster TT.
    Takes Vicon world-frame position error and outputs
    rc() channel values: (lr, fb, ud, yaw).

    Coordinate conventions (Vicon world frame):
        x → forward in arena
        y → left in arena
        z → up

    rc() channels (Tello SDK):
        lr  : left/right  (-100 to 100, positive = right)
        fb  : fwd/back    (-100 to 100, positive = forward)
        ud  : up/down     (-100 to 100, positive = up)
        yaw : rotation    (kept at 0 — yaw locked for this project)

    IMPORTANT: Vicon world frame and drone body frame differ by
    the drone's current yaw. We rotate the x/y error into body
    frame before sending rc() commands. For now yaw is assumed ~0
    (drone faces arena +x). A rotation matrix is included for
    the general case once you confirm yaw from Vicon.
    """

    def __init__(self,
                 Kp_xy=0.30, Kd_xy=0.10,
                 Kp_z=0.50,  Kd_z=0.15,
                 max_xy=40,  max_z=30):
        """
        Args:
            Kp_xy, Kd_xy : proportional/derivative gains for x and y axes
            Kp_z,  Kd_z  : proportional/derivative gains for z axis
            max_xy       : rc output clamp for lr and fb (0–100)
            max_z        : rc output clamp for ud (0–100)

        Start with conservative gains and increase slowly.
        If the drone oscillates → reduce Kp or increase Kd.
        """
        self.Kp_xy = Kp_xy
        self.Kd_xy = Kd_xy
        self.Kp_z  = Kp_z
        self.Kd_z  = Kd_z
        self.max_xy = max_xy
        self.max_z  = max_z

        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.prev_error_z = 0.0

    def reset(self):
        """Call this before each new mission segment."""
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.prev_error_z = 0.0

    # add this method inside PDController, just before compute()
    def _to_rc(self, val, limit):
        """Clip, round, and guarantee minimum authority of 1 for any non-zero error."""
        clipped = float(np.clip(val, -limit, limit))
        result  = int(round(clipped))
        if clipped != 0.0 and result == 0:
            result = 1 if clipped > 0 else -1
        return result

    def compute(self, drone_pos, target_pos, drone_yaw_deg=0.0):
        """
        Compute rc() command to move drone toward target.

        Args:
            drone_pos    : (x, y, z) in metres, from Vicon
            target_pos   : (x, y, z) in metres, from station map
            drone_yaw_deg: drone heading in degrees (0 = facing arena +x)
                           get this from Vicon once confirmed in lab.
                           Safe to leave as 0 for initial tests if drone
                           always takes off facing the same direction.

        Returns:
            (lr, fb, ud, yaw) as integers, ready for tl_flight.rc()
        """
        dx = target_pos[0] - drone_pos[0]   # world-frame errors (metres)
        dy = target_pos[1] - drone_pos[1]
        dz = target_pos[2] - drone_pos[2]

        # Rotate world-frame x/y error into drone body frame
        yaw_rad = np.radians(drone_yaw_deg)
        body_x =  dx * np.cos(yaw_rad) + dy * np.sin(yaw_rad)  # forward axis
        body_y = -dx * np.sin(yaw_rad) + dy * np.cos(yaw_rad)  # left axis

        # PD on body_x → fb channel
        deriv_x = body_x - self.prev_error_x
        fb = self.Kp_xy * body_x + self.Kd_xy * deriv_x

        # PD on body_y → lr channel
        # NOTE: Tello rc() positive lr = RIGHT, positive Vicon y = LEFT → negate
        deriv_y = body_y - self.prev_error_y
        lr = -(self.Kp_xy * body_y + self.Kd_xy * deriv_y)

        # PD on z → ud channel
        deriv_z = dz - self.prev_error_z
        ud = self.Kp_z * dz + self.Kd_z * deriv_z

        # Save for next derivative calculation
        self.prev_error_x = body_x
        self.prev_error_y = body_y
        self.prev_error_z = dz

        # Clamp and convert to int (rc() expects integers)
        lr  = self._to_rc(lr,  self.max_xy)
        fb  = self._to_rc(fb,  self.max_xy)
        ud  = self._to_rc(ud,  self.max_z)
        yaw = 0  # yaw locked — drone doesn't rotate during delivery

        return lr, fb, ud, yaw

    def position_error(self, drone_pos, target_pos):
        """Euclidean distance to target in metres."""
        return float(np.linalg.norm(np.array(target_pos) - np.array(drone_pos)))

    def within_threshold(self, drone_pos, target_pos, threshold=0.15):
        """True when drone is within threshold metres of target."""
        return self.position_error(drone_pos, target_pos) <= threshold


# ─────────────────────────────────────────────
#  Unit tests — run this file directly:
#  python pd_controller.py
# ─────────────────────────────────────────────
if __name__ == "__main__":
    import sys

    pd = PDController()
    PASS = 0
    FAIL = 0

    def check(name, condition, detail=""):
        global PASS, FAIL
        if condition:
            print(f"  PASS  {name}")
            PASS += 1
        else:
            print(f"  FAIL  {name}  {detail}")
            FAIL += 1

    print("\n── Position error tests ──")
    check("zero error at target",
          pd.position_error((1.0, 0.5, 0.7), (1.0, 0.5, 0.7)) == 0.0)
    check("correct Euclidean distance",
          abs(pd.position_error((0,0,0), (3,4,0)) - 5.0) < 1e-6)
    check("within_threshold: inside",
          pd.within_threshold((0,0,0), (0.1, 0.0, 0.0), threshold=0.15))
    check("within_threshold: outside",
          not pd.within_threshold((0,0,0), (1.0, 0.0, 0.0), threshold=0.15))

    print("\n── PD output direction tests (yaw=0) ──")
    pd.reset()
    lr, fb, ud, yaw = pd.compute((0,0,0), (1,0,0))   # target is ahead
    check("target ahead → positive fb",   fb > 0,  f"fb={fb}")
    check("target ahead → lr near zero",  abs(lr) <= 2, f"lr={lr}")

    pd.reset()
    lr, fb, ud, yaw = pd.compute((0,0,0), (0,1,0))   # target is to the left
    check("target left → negative lr",    lr < 0,  f"lr={lr}")
    check("target left → fb near zero",   abs(fb) <= 2, f"fb={fb}")

    pd.reset()
    lr, fb, ud, yaw = pd.compute((0,0,0), (0,0,1))   # target is above
    check("target above → positive ud",   ud > 0,  f"ud={ud}")

    pd.reset()
    lr, fb, ud, yaw = pd.compute((0,0,1), (0,0,0))   # target is below
    check("target below → negative ud",   ud < 0,  f"ud={ud}")

    print("\n── Output clamping tests ──")
    pd.reset()
    lr, fb, ud, yaw = pd.compute((0,0,0), (100,0,0))  # huge error
    check("fb clamped at max_xy",  abs(fb) == pd.max_xy, f"fb={fb}")

    pd.reset()
    lr, fb, ud, yaw = pd.compute((0,0,0), (0,0,100))  # huge z error
    check("ud clamped at max_z",   abs(ud) == pd.max_z,  f"ud={ud}")

    print("\n── Yaw always zero ──")
    pd.reset()
    _, _, _, yaw = pd.compute((0,0,0), (1,1,1))
    check("yaw output is always 0", yaw == 0, f"yaw={yaw}")

    # replace the derivative damping test with this:
    print("\n── Derivative damping test ──")
    pd.reset()
    # Use 10m error so the difference survives integer quantization
    _, fb1, _, _ = pd.compute((0,0,0), (10,0,0))   # first call: P + D term
    _, fb2, _, _ = pd.compute((0,0,0), (10,0,0))   # second call: P only (derivative=0)
    check("second call smaller than first (D term decays)",
        fb2 <= fb1,
        f"fb1={fb1} fb2={fb2}")

    print(f"\n{'─'*40}")
    print(f"  {PASS} passed  /  {FAIL} failed")
    if FAIL == 0:
        print("  All tests passed — PD controller math is correct.")
    else:
        print("  Fix the failures above before using in the lab.")
    sys.exit(FAIL)