from dataclasses import dataclass, field
from lerobot.cameras import  CameraConfig, Cv2Rotation
from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.cameras.zmq import ZMQCameraConfig
from lerobot.robots.config import RobotConfig


def sim_cameras() -> dict[str, CameraConfig]:
    # Names must match the <camera> names in the MuJoCo scene and mj_sim.py's CAMERAS
    return {
        name: ZMQCameraConfig(server_address="127.0.0.1", port=5555, camera_name=name,
                              fps=30, width=640, height=480)
        for name in ("top_down_cam", "wrist_cam")
    }


@RobotConfig.register_subclass("my_parol6")
@dataclass
class myParol6Config(RobotConfig):
    host: str = "127.0.0.1"
    port: int = 5005
    # True: read cameras from mj_sim.py's MuJoCo render stream instead of USB cameras
    sim: bool = False
    cameras: dict[str, CameraConfig] = field(
        default_factory=lambda: {
            "cam_1": OpenCVCameraConfig(
                index_or_path=0,
                fps=30,
                width=1280,
                height=720,
                rotation=Cv2Rotation.ROTATE_180,
            ),
            "cam_2": OpenCVCameraConfig(
                index_or_path=2,
                # Iriun on AVFoundation always opens 720p at 60 fps; recording still samples at 30
                fps=60,
                width=1280,
                height=720,
            ),
        }
    )

    def __post_init__(self):
        if self.sim:
            self.cameras = sim_cameras()
        super().__post_init__()


# lerobot-rollout \
#   --strategy.type=base \
#   --policy.path=lerobot/pi0_base \
#   --policy.empty_cameras=1 \
#   --policy.dtype=bfloat16 \
#   --policy.num_inference_steps=5 \
#   --device=mps \
#   --robot.type=my_parol6 \
#   --robot.host=127.0.0.1 \
#   --rename_map='{"observation.images.cam_1": "observation.images.base_0_rgb", "observation.images.cam_2": "observation.images.left_wrist_0_rgb"}' \
#   --task="put the green cube on the red cube" \
#   --duration=3000 
