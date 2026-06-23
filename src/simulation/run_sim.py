"""
run_sim.py -- Launch the Dal Robotics quadruped simulation.

Usage:
    python src/simulation/run_sim.py

Controls (MuJoCo viewer):
    Left click + drag    Rotate camera
    Right click + drag   Pan camera
    Scroll wheel         Zoom
    Space                Pause / unpause
    Backspace            Reset simulation
    Ctrl+R               Reload model (useful while editing the XML)
    Double-click body    Select and highlight a body
    Escape               Exit
"""

import os
import time
import numpy as np
import mujoco
import mujoco.viewer

# Path to the MJCF model relative to this file
MODEL_PATH = os.path.join(os.path.dirname(__file__), "models", "quadruped.xml")


def load_model():
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data  = mujoco.MjData(model)
    return model, data


def apply_standing_pose(model, data):
    """
    Set the robot into a crouched standing pose using the 'stand' keyframe
    defined in quadruped.xml. This keeps the feet on the ground and the body
    at a stable height.
    """
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "stand")
    if key_id >= 0:
        mujoco.mj_resetDataKeyframe(model, data, key_id)
    mujoco.mj_forward(model, data)


def simple_pd_controller(model, data, targets, kp=30.0, kd=1.0):
    """
    Simple PD controller: drives all joints toward `targets` (dict of joint_name -> angle_rad).
    This is the sim equivalent of what motor_tuner.py does on real hardware.
    """
    for joint_name, target_pos in targets.items():
        joint_id  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        act_id    = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, joint_name + "_act")
        if joint_id < 0 or act_id < 0:
            continue
        qpos_adr  = model.jnt_qposadr[joint_id]
        qvel_adr  = model.jnt_dofadr[joint_id]
        pos_err   = target_pos - data.qpos[qpos_adr]
        vel       = data.qvel[qvel_adr]
        torque    = kp * pos_err - kd * vel
        torque    = np.clip(torque, -18.0, 18.0)  # AK40-10 torque limit
        data.ctrl[act_id] = torque


# Standing pose: hip_fe slightly forward, knee bent to create a stable crouch.
# These angles are approximate -- update when Mechanical provides exact link lengths.
STAND_TARGETS = {
    "FL_hip_ab": 0.0,  "FL_hip_fe":  0.65, "FL_knee": -1.3,
    "FR_hip_ab": 0.0,  "FR_hip_fe":  0.65, "FR_knee": -1.3,
    "RL_hip_ab": 0.0,  "RL_hip_fe":  0.65, "RL_knee": -1.3,
    "RR_hip_ab": 0.0,  "RR_hip_fe":  0.65, "RR_knee": -1.3,
}


def main():
    print("Loading model:", MODEL_PATH)
    model, data = load_model()
    print(f"Model loaded: {model.nbody} bodies, {model.njnt} joints, {model.nu} actuators")

    apply_standing_pose(model, data)
    print("Standing pose applied. Opening viewer...")
    print()
    print("Controls: Space=pause  Backspace=reset  Ctrl+R=reload  Esc=quit")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            # Apply PD controller to hold standing pose
            simple_pd_controller(model, data, STAND_TARGETS)

            mujoco.mj_step(model, data)
            viewer.sync()


if __name__ == "__main__":
    main()
