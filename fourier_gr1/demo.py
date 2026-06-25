import os
import mujoco
import mujoco.viewer
import time
from pathlib import Path

# Run with: mjpython demo.py
# note: "python3 demo.py" won't work
# Press "0" in MUJOCO to remove "collision wrap bodies", then you'll see a pure GR1 robot

urdf_dir = Path("./GR1/gr1t1/basic_urdf")
os.chdir(urdf_dir)

model = mujoco.MjModel.from_xml_path("gr1t1.urdf")
data = mujoco.MjData(model)

print("ngeom:", model.ngeom)
print("nmesh:", model.nmesh)

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(model.opt.timestep)