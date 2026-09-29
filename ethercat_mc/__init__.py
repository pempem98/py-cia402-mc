"""Multi-axis EtherCAT motion control for CiA 402 drives (eRob, Maxon).

Typical use:

    from ethercat_mc import MotionController, load_config

    cfg = load_config("configs/robot.yaml")
    with MotionController(cfg) as mc:
        mc.start()
        mc.enable_all()
        mc.axis("joint1").move_to(45.0)
"""
from . import cia402
from .axis import Axis, AxisError, AxisState
from .config import AxisConfig, BusConfig, load as load_config
from .controller import MotionController
from .master import BusError, EtherCATMaster, find_adapters

__version__ = "0.1.0"

__all__ = [
    "Axis",
    "AxisConfig",
    "AxisError",
    "AxisState",
    "BusConfig",
    "BusError",
    "EtherCATMaster",
    "MotionController",
    "cia402",
    "find_adapters",
    "load_config",
]
