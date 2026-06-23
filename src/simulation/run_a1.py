"""
run_a1.py -- Launch the Unitree A1 quadruped simulation (MuJoCo Menagerie model).

The A1 is similar in form factor to the robot we are building. Use this for
gait research and algorithm development until the custom quadruped.xml is
updated with real dimensions from the Mechanical team.

Usage:
    python src/simulation/run_a1.py

Controls:
    Left click + drag    Rotate camera
    Right click + drag   Pan
    Scroll               Zoom
    Space                Pause / unpause
    Backspace            Reset
    Esc                  Quit
"""

import os
import mujoco
import mujoco.viewer

SCENE_PATH = os.path.join(os.path.dirname(__file__), "models", "unitree_a1", "scene.xml")


def main():
    print("Loading Unitree A1 model...")
    model = mujoco.MjModel.from_xml_path(SCENE_PATH)
    data  = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    print(f"  Bodies   : {model.nbody}")
    print(f"  Joints   : {model.njnt}")
    print(f"  Actuators: {model.nu}")
    print("Opening viewer -- Space=pause  Esc=quit")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()


if __name__ == "__main__":
    main()
