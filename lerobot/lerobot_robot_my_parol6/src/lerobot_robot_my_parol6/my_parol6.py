import json, socket, threading, time
from lerobot.robots.robot import Robot
from lerobot.cameras.utils import make_cameras_from_configs

from .config_my_parol6 import myParol6Config

class  myParol6(Robot):
    config_class = myParol6Config
    name = "my_parol6"
    def __init__(self, config):
        super().__init__(config)
        self.cfg = config
        self.cameras = make_cameras_from_configs(config.cameras)
        self._state = None
        self._lock = threading.Lock()

    @property
    def _motors_ft(self) -> dict[str, type]:
        return {
            "joint_1.pos": float,
            "joint_2.pos": float,
            "joint_3.pos": float,
            "joint_4.pos": float,
            "joint_5.pos": float,
            "joint_6.pos": float,
            "gripper.pos": float,
        }

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.cameras[cam].height, self.cameras[cam].width, 3) for cam in self.cameras
        }

    @property
    def observation_features(self) -> dict:
        return {**self._motors_ft, **self._cameras_ft}

    @property
    def action_features(self) -> dict:
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        return self._state is not None

    @property
    def is_calibrated(self): return True
    def calibrate(self): pass
    def configure(self): pass   

    def connect(self, calibrate = True):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(1.0)
        # Resolve once; a .local name would otherwise hit mDNS on every sendto
        self._addr = (socket.gethostbyname(self.cfg.host), self.cfg.port)
        self.send_cmd("ping")
        self._running = True
        threading.Thread(target=self._rx, daemon=True).start()
        for cam in self.cameras.values(): cam.connect()
        while self._state is None: pass  # wait for the first status packet

    def send_cmd(self, cmd: str):
        """Sends commands to the C++ backend"""
        self.sock.sendto(cmd.encode("utf-8"), self._addr)
    
    def _rx(self):
        last_ping = time.monotonic()
        while self._running:
            # Controller drops clients 5 s after they last *sent* something, even while it is streaming to them
            if time.monotonic() - last_ping > 2.0:
                self.send_cmd("ping"); last_ping = time.monotonic()
            try: data, _ = self.sock.recvfrom(4096)
            except socket.timeout:
                self.send_cmd("ping"); last_ping = time.monotonic(); continue
            except OSError:
                break                                    # socket closed by disconnect()
            if data[:1] == b"{":
                try:
                    msg = json.loads(data)
                except json.JSONDecodeError:
                    # e.g. a NaN in the C++ printf output; one bad packet must not kill this thread
                    print(f"[my_parol6] bad status packet: {data[:80]!r}")
                    continue
                with self._lock: self._state = msg

    def get_observation(self) -> dict[str, any]:
        if not self.is_connected:
            raise ConnectionError(f"{self} is not connected.")
        with self._lock: s = self._state
        self._last_obs_status = s

        obs = {f"joint_{i + 1}.pos": float(s["pos"][i]) for i in range(6)}
        obs["gripper.pos"] = s["grip_ack"] / 140 
        for name, cam in self.cameras.items():
            obs[name] = cam.async_read()
        return obs

    def get_status(self) -> dict:
        """Latest raw status packet from the controller."""
        with self._lock: return dict(self._state)

    def get_commanded_action(self) -> dict[str, float]:
        """Controller's IK joint target + gripper command from the same packet as the last get_observation()."""
        s = self._last_obs_status
        action = {f"joint_{i + 1}.pos": float(s["tgt"][i]) for i in range(6)}
        action["gripper.pos"] = s["grip"] / 140
        return action

    def send_action(self, action) -> dict[str, any]:
        if not self.is_connected:
            raise ConnectionError(f"{self} is not connected.")
        q = [action[f"joint_{i}.pos"] for i in range(1, 7)]
        grip = min(140, max(0, round(action["gripper.pos"] * 140)))
        self.send_cmd(" ".join(f"{v:.3f}" for v in q))
        self.send_cmd(f"gripper {grip}")
        return action

    def disconnect(self):
        if not self.is_connected:
            raise ConnectionError(f"{self} is not connected.")
        self._running = False
        self.send_cmd("stop")
        for cam in self.cameras.values(): cam.disconnect()
        self.sock.close()
        with self._lock: self._state = None
