"""Import and solve only; no rclpy.init, Node, DDS, camera or robot access."""
import importlib
import json
import platform
import sys
from pathlib import Path
import numpy as np
import casadi
from vln_mpc.mpc import MPCController

assert Path(casadi.__file__).resolve().is_relative_to(Path(__file__).resolve().parent/'site-packages')
assert casadi.__version__ == '3.8.2'

modules = {}
for name in ('rclpy', 'geometry_msgs.msg', 'nav_msgs.msg', 'std_msgs.msg',
             'numpy', 'casadi', 'vln_mpc.mpc_node'):
    module = importlib.import_module(name)
    modules[name] = module.__file__
controller = MPCController(horizon=5, dt_s=.1, w_max=.4, a_max_v=2.,
    a_max_w=5., q_weights=(10., 10., 1.), r_weights=(.1, .1))
reference = np.array([[.05*i, 0., 0.] for i in range(1, 6)])
command, prediction = controller.solve(np.zeros(3), reference, (0., 0.), .2)
assert np.all(np.isfinite(prediction)) and 0. < command[0] <= .2
print(json.dumps(dict(python=sys.executable, version=sys.version,
    architecture=platform.machine(), casadi=casadi.__version__,
    numpy=np.__version__, modules=modules, solve_command=command,
    ros_initialized=False), indent=2))
